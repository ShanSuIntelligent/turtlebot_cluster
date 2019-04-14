/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>


int main(int argc,char* argv[])
{
  std::string file_path = argv[1];
  std::cout<<file_path<<std::endl;
  std::ifstream in(file_path);
  std::string data;
  for(std::string s;in>>s;)
  {
    data.append(s);
    data.append(",");
  }
  std::vector<std::string> poses;
  std::string out;
  if(data.find('---') == std::string::npos)
    return false;
  boost::split(poses,data,boost::is_any_of("---"));
  for(std::string pose : poses) {
    boost::regex reg_space(":");
    pose = boost::regex_replace(pose,reg_space,"");
    out.append(pose);
    out.append("\n");
//    std::cout<<pose.c_str()<<std::endl;
  }
  std::ofstream outfile(argv[2]);
  for(char c : out){
    outfile.put(c);
  }
  outfile.close();
  return 0;
}
