#include "yaml-cpp/yaml.h"
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
//-lyaml-cpp

int main()
{
	
  YAML::Node lineup = YAML::LoadFile("pololu.yaml");
  
	for (std::size_t i=0; i<lineup.size(); i++) 
	{
		std::cout << lineup[i]["id"].as<std::string>() << "\n";
	}
  return 0;
}
