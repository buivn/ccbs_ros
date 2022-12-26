#include <ccbs/task.h>
Task::Task()
{
    agents.clear();
}

bool Task::get_task(const char *FileName, int k)
{
    tinyxml2::XMLElement *root = 0, *agent = 0;
    tinyxml2::XMLDocument doc;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (agent = root->FirstChildElement(); agent; agent = agent->NextSiblingElement())
    {
        Agent a;
        a.start_i = agent->DoubleAttribute(CNS_TAG_START_I);
        a.start_j = agent->DoubleAttribute(CNS_TAG_START_J);
        a.start_id = agent->IntAttribute(CNS_TAG_START_ID);
        a.goal_i = agent->DoubleAttribute(CNS_TAG_GOAL_I);
        a.goal_j = agent->DoubleAttribute(CNS_TAG_GOAL_J);
        a.goal_id = agent->IntAttribute(CNS_TAG_GOAL_ID);
        a.id = int(agents.size());
        agents.push_back(a);
        if(int(agents.size()) == k)
            break;
    }
    return true;
}

bool Task::set_new_task(std::vector<std::pair<double, double>> starts, std::vector<std::pair<double, double>> goals,
            std::vector<int> start_ids,std::vector<int> goal_ids) 
{
    for(unsigned int i = 0; i < starts.size(); i++)
    {
        Agent a;
        a.start_i = starts[i].first;
        a.start_j = starts[i].second;
        a.start_id = start_ids[i];
        a.goal_i = goals[i].first;
        a.goal_j = goals[i].second;
        a.goal_id = goal_ids[i];
        agents.push_back(a);
    }
    return true;
}
void Task::make_ij(const Map& map)
{
    for(unsigned int i = 0; i < agents.size(); i++)
    {
        gNode start = map.get_gNode(agents[i].start_id), goal = map.get_gNode(agents[i].goal_id);
        agents[i].start_i = start.i;
        agents[i].start_j = start.j;
        agents[i].goal_i = goal.i;
        agents[i].goal_j = goal.j;
    }

}

Agent Task::get_agent(int id) const
{
    if(id >= 0 && id < int(agents.size()))
        return agents[id];
    else
        return Agent();
}
