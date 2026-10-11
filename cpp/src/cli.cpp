#include "embodied/core.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
int main(int argc,char** argv) {
 embodied::JointVector q;
 if(argc==2 && std::string(argv[1])=="--benchmark") {
  q<<0,0,0,-1.57079,0,1.57079,-.7853;
  std::vector<double> samples; samples.reserve(10000);double checksum=0;
  for(int i=0;i<10000;++i) {
   q[0]=i*.00001;
   auto start=std::chrono::steady_clock::now();
   auto p=embodied::panda_fk(q);auto J=embodied::panda_jacobian(q);
   auto end=std::chrono::steady_clock::now();
   checksum+=p.position.x()+J(0,0);
   samples.push_back(std::chrono::duration<double,std::micro>(end-start).count());
  }
  std::sort(samples.begin(),samples.end());
  std::cout<<"{\"samples\":10000,\"fk_jacobian_us\":{\"p50\":"<<samples[5000]
           <<",\"p95\":"<<samples[9500]<<",\"p99\":"<<samples[9900]<<"},\"checksum\":"<<checksum<<"}\n";
  return 0;
 }
 if(argc!=8) { std::cerr<<"usage: core_cli q1 ... q7\n"; return 2; }
 try {
  for(int i=0;i<7;++i) q[i]=std::stod(argv[i+1]);
  auto p=embodied::panda_fk(q);auto J=embodied::panda_jacobian(q);
  std::cout<<std::setprecision(17);
  for(int i=0;i<3;++i) std::cout<<p.position[i]<<' ';
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) std::cout<<p.rotation(r,c)<<' ';
  for(int r=0;r<6;++r) for(int c=0;c<7;++c) std::cout<<J(r,c)<<' ';
  std::cout<<'\n';
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
