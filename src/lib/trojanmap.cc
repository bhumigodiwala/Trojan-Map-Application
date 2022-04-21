
#include "trojanmap.h"
#include<string>
#include <utility>
#include <ctype.h>
#include<iostream>
#include<type_traits>
//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
    return data[id].lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
    return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    return data[id].neighbors;
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string res = "";
  res = data[name].id;
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);

  if(name == "") { //if empty name then return empty 
    return results;
  }
  bool find_location = false;
  for (auto it = data.begin(); it != data.end() && !find_location; ++it)
  {
    std::string current = it->second.name;

    // std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    // std::transform(current.begin(), current.end(), current.begin(), ::tolower);
    
    if (name.compare(current) == 0)
    { //check if input name matches currentMap name
      results.first = it->second.lat;
      results.second = it->second.lon;
      find_location = true;
    }
  }
  return results;

  // return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */

int TrojanMap::CalculateEditDistance(std::string a, std::string b){
  int len1 = a.size();
  int len2 = b.size();

  int arr[2][len1 + 1];

  memset(arr, 0, sizeof arr);

  for (int i = 0; i <= len1; i++){
    arr[0][i] = i;
  }

  for (int i = 1; i <= len2; i++) {
    for (int j = 0; j <= len1; j++) {
      if (j == 0){
        arr[i % 2][j] = i;
      }
      else if (a[j - 1] == b[i - 1]) {
        arr[i % 2][j] = arr[(i - 1) % 2][j - 1];
      }
      else {
        arr[i % 2][j] = 1 + std::min(arr[(i - 1) % 2][j], std::min(arr[i % 2][j - 1], arr[(i - 1) % 2][j - 1]));
      }
    }
  }

  return arr[len2 % 2][len1];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp;
  
  // std::cout << TrojanMap::Autocomplete(name);
  bool find_location= false;
  int min_dis = INT_MAX;
  for (auto it = data.begin(); it != data.end() && !find_location; ++it)
  {
    std::string current = it->second.name;
    std::string curr_1 = current;

    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    std::transform(curr_1.begin(), curr_1.end(), curr_1.begin(), ::tolower);
    
    if (TrojanMap::CalculateEditDistance(name, curr_1) <= min_dis)
    { //check if input name matches currentMap name
      tmp = current;
      min_dis = TrojanMap::CalculateEditDistance(name, curr_1);
    }
  }

  // for (auto i:Autocomplete(name)){
  // TrojanMap::CalculateEditDistance(name);
  //   tmp.push_back(i);
  // }
  return tmp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;

  if(name == "") { 
    return results;
  }
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    std::string current = it->second.name;
    if (name.size() <= current.size())
    {                                                             
      std::string tempBeginning = current.substr(0, name.size()); 

      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      
      std::transform(tempBeginning.begin(), tempBeginning.end(), tempBeginning.begin(), ::tolower);

      if (name.compare(tempBeginning) == 0)
      { 
        results.push_back(current);
      }
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;  

  double INF = 999999999999999999L;

  std::map< const Node*, double> costs;
  std::map< const Node*, std::string> from;
  std::map< const Node*, bool> visited;

  std::string src_id, dest_id;

  for(auto it = data.begin(); it != data.end(); it++){
    costs[&it->second] = INF;

    if (it->second.name == location1_name){
      src_id = it->second.id;
    }
    if (it->second.name == location2_name){
      dest_id = it->second.id;
    }
  }

  auto compare = [&costs](const Node *a, const Node *b){
    return costs[a] > costs[b];
  };

  std::priority_queue<const Node*, std::vector<const Node*>, decltype(compare) > queue(compare);

  const Node *curr_node = &data[src_id];
  costs[curr_node] = 0;

  queue.push(curr_node);

  while(!queue.empty()){
    curr_node = queue.top();
    queue.pop();


    if(curr_node->id == dest_id){
      std::string temp = dest_id;
      while (temp != src_id)
      {
        path.insert(path.begin(), temp);
        temp = from[&data[temp]];
      }
      
      
    path.insert(path.begin(), src_id);//add those parent nodes in the front of x vector (a.k.a src_node position)
      return path;
    }
    //if we didn't find it
    visited[curr_node] = true; //assume curr_node has been visited

    for (const std::string &adj_node : curr_node->neighbors) { //loop through and find all adjacent nodes of curr_node
      Node *dest_node = &data[adj_node]; //Extract every nodes that points to the location of curr_node from the map, "data"
      // dest_node --> adj_node
      if (visited[dest_node]) { //If the node is visited
        continue; //skip the visited node and continue
      }
      
      double weight = CalculateDistance(curr_node->id, dest_node->id); //weight(curr_node, dest_node)
      if (costs[dest_node] > costs[curr_node] + weight) { //If there is a shorter path to dest_node through curr_node
        from[dest_node] = curr_node->id;//updating its prev_nodes of shortest path (a.k.a minimum costs)
        
        costs[dest_node] = costs[curr_node] + weight; //Updating current distance from curr_node to dest_node
        queue.push(dest_node); //push it into the queue
      }
    }
  }

  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  // std::vector<std::string> path;
  // return path;

  std::string src_id, dest_id;
  for(auto node: data) {
    if(node.second.name == location1_name)
    {
      src_id = node.first;
    }
    if(node.second.name == location2_name)
    {
      dest_id = node.first;
    }
  }
  Node node1 = data[src_id];
  Node node2 = data[dest_id];
  if(node1.id=="" || node2.id=="") return {};
  if(node1.id == node2.id) return {node1.id};

  std::unordered_map<std::string,double> weights_matrix; //Weights Map
  std::unordered_map<std::string,double> distance_To;
  std::unordered_map<std::string,std::string> Parent_Node;

  distance_To[node1.id] = 0;
  bool changed = true;
  int time = 0;
  while(changed && time<data.size()-1){
    time++;
    changed = false;
    for(auto &node_dist:distance_To){
      Node curr_node = data[node_dist.first];
      for(std::string &adj : curr_node.neighbors){
        if(data.count(adj) == 0)
          continue; // In case there is no such Node, invalid data point

        // Generate key for the Link Weight Map
        std::string weight_map_key = curr_node.id;
        weight_map_key.append(",");
        weight_map_key.append(adj);

        if(weights_matrix.count(weight_map_key)==0){
          // Generate another key for the Link Weight Map
          std::string key_alt = adj;
          key_alt.append(",");
          key_alt.append(curr_node.id);
          weights_matrix[weight_map_key] = CalculateDistance(curr_node.id, data[adj].id);
          weights_matrix[key_alt] = weights_matrix[weight_map_key];
        }

        if(distance_To.count(adj)==0 || distance_To[adj] > weights_matrix[weight_map_key]+distance_To[curr_node.id] ){
          distance_To[adj] = weights_matrix[weight_map_key] + distance_To[curr_node.id];
          Parent_Node[adj] = curr_node.id;
          changed = true;
        }
      }
    }
  }
  
  if(Parent_Node.count(node2.id)==0)  return {};

  std::vector<std::string> path;
  std::string id_pointer = node2.id;
  while(id_pointer!=node1.id){
    path.push_back(id_pointer);
    id_pointer = Parent_Node[id_pointer];
  }
  path.push_back(id_pointer);

  std::reverse(path.begin(), path.end());
  return path;
  
}


/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  return result;                                                     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  double min_lat, max_lat, min_lon, max_lon;
  min_lat = square[0];
  max_lat = square[1];

  min_lon = square[2];
  max_lon = square[3];
for (auto it = data.begin(); it != data.end(); ++it)
  {
    if (it->second.id == id){
  double current_lat = it->second.lat;
    double current_lon = it->second.lon;
    

  if ( (current_lat>=min_lat && current_lat<=max_lat ) && (current_lon>=min_lon && current_lon<=max_lon) )
      return true;
      }
}
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;

  double min_lat, max_lat, min_lon, max_lon;

  min_lat = square[0];
  max_lat = square[1];

  min_lon = square[2];
  max_lon = square[3];

  for (auto it = data.begin(); it != data.end(); ++it)
  {
    double current_lat = it->second.lat;
    double current_lon = it->second.lon;

    if ((current_lat>=min_lat && current_lat<=max_lat ) && (current_lon<=min_lon && current_lon>=max_lon) )
      subgraph.push_back(it->second.id);
    
  }
  
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {

  
   
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
