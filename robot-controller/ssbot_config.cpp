#include "ssbot_config.h"

using namespace std;

map<string, string> parameters;

void read_parameters()
{
	fstream f;
	const char* config_env = std::getenv("AUTOSURGVIEW_CONFIG");
	f.open(config_env ? config_env : "robot-controller/config/parameters.txt");
	if (!f.is_open())
		throw runtime_error("Cannot open config. Copy parameters.example.txt to parameters.txt or set AUTOSURGVIEW_CONFIG.");

	string name, value;

	while (f >> name)
	{
		if (!name.empty() && name[0] == '#')
		{
			string ignored;
			getline(f, ignored);
			continue;
		}
		if (!(f >> value)) break;
		parameters[name] = value;
	}

	f.close();
}
