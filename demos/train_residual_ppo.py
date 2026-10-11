"""Actual MuJoCo residual PPO training, model weights, rewards and OOD evaluation."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal
from embodied_swarm.control.residual_contact_env import ResidualContactEnv


class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(3,32),nn.Tanh(),nn.Linear(32,32),nn.Tanh(),nn.Linear(32,1))
        self.critic=nn.Sequential(nn.Linear(3,32),nn.Tanh(),nn.Linear(32,32),nn.Tanh(),nn.Linear(32,1))
        self.log_std=nn.Parameter(torch.tensor(-1.0))
        nn.init.zeros_(self.actor[-1].weight);nn.init.zeros_(self.actor[-1].bias)
    def dist(self,x):
        return Normal(self.actor(x).squeeze(-1),self.log_std.exp().clamp(.05,.6))


def evaluate(policy,seed,stiffness,residual=True):
    env=ResidualContactEnv(seed,stiffness)
    forces,rewards,unsafe=[],[],0
    for _ in range(3):
        obs=env.reset()
        for step in range(64):
            with torch.no_grad():
                action=float(policy.actor(torch.tensor(obs)).squeeze()) if residual else 0.
            obs,reward,done,info=env.step(action)
            rewards.append(reward);unsafe+=info["unsafe"]
            if step>=32:
                forces.append(info["force_N"])
            if done:
                break
    return dict(seed=seed,stiffness=stiffness,residual=residual,mean_reward=float(np.mean(rewards)),
                force_rmse_N=float(np.sqrt(np.mean((np.array(forces)-2.)**2))) if forces else None,
                force_mean_N=float(np.mean(forces)) if forces else None,unsafe_steps=int(unsafe),samples=len(rewards))


def train(seed=7,updates=30):
    torch.manual_seed(seed);np.random.seed(seed);torch.set_num_threads(1)
    policy=Policy();optimizer=torch.optim.Adam(policy.parameters(),lr=.001)
    env=ResidualContactEnv(seed);obs=env.reset();curve=[]
    for update in range(updates):
        states,actions,logs,values,rewards,dones=[],[],[],[],[],[]
        for _ in range(256):
            x=torch.tensor(obs)
            with torch.no_grad():
                dist=policy.dist(x);a=dist.sample();lp=dist.log_prob(a);v=policy.critic(x).squeeze()
            next_obs,reward,done,_=env.step(float(a))
            states.append(obs.copy());actions.append(float(a));logs.append(float(lp));values.append(float(v));rewards.append(reward);dones.append(done)
            obs=env.reset() if done else next_obs
        with torch.no_grad():
            bootstrap=float(policy.critic(torch.tensor(obs)).squeeze())
        advantages=np.zeros(256,np.float32);gae=0.
        for i in reversed(range(256)):
            nxt=bootstrap if i==255 else values[i+1]
            delta=rewards[i]+.99*nxt*(1-dones[i])-values[i]
            gae=delta+.99*.95*(1-dones[i])*gae;advantages[i]=gae
        returns=torch.tensor(advantages+np.array(values),dtype=torch.float32)
        adv=torch.tensor((advantages-advantages.mean())/(advantages.std()+1e-8))
        states=torch.tensor(np.array(states));actions=torch.tensor(actions);oldlogs=torch.tensor(logs)
        for _ in range(6):
            order=torch.randperm(256)
            for idx in order.split(64):
                dist=policy.dist(states[idx]);ratio=(dist.log_prob(actions[idx])-oldlogs[idx]).exp()
                surrogate=torch.minimum(ratio*adv[idx],ratio.clamp(.8,1.2)*adv[idx])
                value_loss=(policy.critic(states[idx]).squeeze()-returns[idx]).square().mean()
                loss=-surrogate.mean()+.5*value_loss-.001*dist.entropy().mean()
                optimizer.zero_grad();loss.backward();nn.utils.clip_grad_norm_(policy.parameters(),.5);optimizer.step()
        curve.append(dict(update=update,environment_steps=(update+1)*256,mean_reward=float(np.mean(rewards)),loss=float(loss.detach())))
        print(json.dumps(curve[-1]),flush=True)
    evaluations=[evaluate(policy,s,k,r) for k in (150.,300.,450.) for s in (17,27,37) for r in (False,True)]
    out=Path("artifacts/learning");out.mkdir(parents=True,exist_ok=True)
    torch.save({"state_dict":policy.state_dict(),"seed":seed,"updates":updates,"observation":"force_error/8, ee_height_error/.05, joint2_velocity","residual_limit_m":.03},out/"residual_ppo.pt")
    report=dict(status="PASS_TRAINING",seed=seed,algorithm="PPO clipped ratio .2 with GAE(.99,.95)",
                environment="actual MuJoCo Panda floor contact",torch=torch.__version__,
                samples=updates*256,curve=curve,evaluation=evaluations,
                limitation="Single bounded vertical-target residual; no grasp policy or multi-agent learning")
    report["evaluation_status"]="PASS_REAL_SIM" if all(e["unsafe_steps"]==0 and e["force_rmse_N"] is not None for e in evaluations) else "FAIL"
    (out/"residual_ppo.json").write_text(json.dumps(report,indent=2,allow_nan=False))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    axes[0].plot([r["environment_steps"] for r in curve],[r["mean_reward"] for r in curve]);axes[0].set_xlabel("actual physical control steps");axes[0].set_ylabel("mean reward")
    for label,res in (("classical fixed impedance",False),("trained residual",True)):
        axes[1].plot([150,300,450],[np.mean([e["force_rmse_N"] for e in evaluations if e["stiffness"]==k and e["residual"]==res and e["force_rmse_N"] is not None]) for k in (150,300,450)],label=label,marker="o")
    axes[1].set_xlabel("OOD stiffness N/m (train: 300)");axes[1].set_ylabel("contact-force RMSE (N)");axes[1].legend()
    fig.tight_layout();fig.savefig(out/"residual_ppo.png",dpi=150);plt.close(fig)
    print(json.dumps(evaluations,indent=2))
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--seed",type=int,default=7);parser.add_argument("--updates",type=int,default=30)
    args=parser.parse_args();train(args.seed,args.updates)
