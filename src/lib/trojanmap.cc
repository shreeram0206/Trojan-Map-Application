#include "trojanmap.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <limits.h>
#include <fstream>
#include <sstream>


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
    auto it = data.find(id);
    if (it == data.end()){
      return -1;
    }
    return it->second.lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    auto it = data.find(id);
    if (it == data.end()){
      return -1;
    } 
    return it->second.lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) {
    auto it = data.find(id);
    if (it == data.end()){
      return "NULL";
    } 
    return it->second.name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    std::vector<std::string> res;
    auto it = data.find(id);
    if (it == data.end()){
      return res;
    }
    return it->second.neighbors;
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
  for (auto &it : data){
    if (it.second.name == name){
      res = it.first;
      return res;
    }
  }
  return "";
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  std::string id = GetID(name);
  auto it = data.find(id);
  if (it == data.end()){
    return results;
  }

  double latitude = GetLat(id);
  double longitude = GetLon(id);
  results.first = latitude;
  results.second = longitude;
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
  std::for_each(a.begin(), a.end(), [](char &c){
    c = ::tolower(c);
  });
  std::for_each(b.begin(), b.end(), [](char &f){
    f = ::tolower(f);
  });
  int length1 = a.size();
  int length2 = b.size();
  int d[length1 + 1][length2 + 1];
 
  for (int i = 0; i <= length1; i++) {
    for (int j = 0; j <= length2; j++) {
      int upper = d[i][j - 1];
      int left = d[i - 1][j];
      int diag = d[i - 1][j - 1];

      if (i == 0){
        d[i][j] = j;
      }
 
      else if (j == 0){
        d[i][j] = i;
      }

      else if (a[i - 1] != b[j - 1]){
        d[i][j] = 1 + std::min(upper, std::min(left, diag));
      }
      else{
        d[i][j] = diag;
      }
    }
  }
  return d[length1][length2];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::for_each(name.begin(), name.end(), [](char &c){  // Convert input name to lowercase.
    c = ::tolower(c);
  });
  std::string tmp = "";
  int minDist = INT_MAX;    // Define min distance value to INT_MAX
  for (auto &nam : data){
    std::string tmpName = nam.second.name;
    std::for_each(tmpName.begin(), tmpName.end(), [](char &d){  // Convert every name in map to lowercase before comparison.
      d = ::tolower(d);
    });
    int dist = CalculateEditDistance(name, nam.second.name);  // Call function to calculate edit distance.
    if (dist < minDist){    // If edit distance is less than minimum distance update minDist.
      minDist = dist;
      tmp = nam.second.name;   // Get name corresponding to minimum edit distance.
    }
  }
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
  std::for_each(name.begin(), name.end(), [](char &c){
    c = ::tolower(c);
  });

  for (auto &x : data){
    std::string temp_name = x.second.name;
    std::for_each(temp_name.begin(), temp_name.end(), [](char &d){
      d = ::tolower(d);
    });
    for(int i = 0; i < name.size(); i++){
      if (name.size() > temp_name.size()){
        break;
      }
      if(temp_name[i] != name[i]){
        break;
      }
      if (i == name.size() - 1){
        results.push_back(x.second.name);
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
  std::string loc_name1 = FindClosestName(location1_name);  // If user incorrectly spells the location, correct it by calling FindClosestName.
  std::string loc_name2 = FindClosestName(location2_name);
  std::string start = GetID(loc_name1);  // Get start node from location name.
  std::string end = GetID(loc_name2);    // Get end node from location name.
  if (start == "" || end == ""){              // Edge Case where start and end are empty strings
    path = {""};
  }
  typedef std::pair<double, std::string> pq;
  std::priority_queue<pq, std::vector<pq>, std::greater<pq>> minheap;  // Create minHeap using priority queue.
  minheap.push(std::make_pair(0, start));    // Put start node into the minHeap.
  std::unordered_map<std::string, double> dist;      // Create unordered map to save shortest distance.
  std::unordered_map<std::string, std::string> pred_map;  // Create unordered map to save predecessors.
  std::unordered_map<std::string, Node>::iterator it;

  for (it = data.begin(); it != data.end(); it++){
    if (data[it->first].id == start){
      dist.insert(std::pair<std::string, double> (start, 0));   // Initialize distance of start node with itself as 0.
    }
    else{
      dist.insert(std::pair<std::string, double> (data[it->first].id, DBL_MAX)); // Initialize distance of other nodes from start node as infinity. 
    }
  }

  while(!minheap.empty()){
    double shortDist = minheap.top().first;   // Find shortest distance node
    std::string prev = minheap.top().second;  // 
    minheap.pop();
    if (prev != end){                         // Check if current node is the end node or not.
      if (shortDist > dist[prev]){            // urrent node’s distance is greater than this node’s current shortest distance in map or not.
        continue;
      }
      else{
        for (auto n : data[prev].neighbors){   // TRaverse current node's neighbors.
          if (dist[n] > shortDist + CalculateDistance(prev, n)){  // Check each neighbor node's distance with previous one.
            dist[n] = shortDist + CalculateDistance(prev, n);   // Update shortest distance map
            minheap.push(std::make_pair(dist[n], n));        // Push pair in minheap
            pred_map[n] = prev;                              // Update predecessor map 
          }
        }

      }
    }
  }
  std::string temp = end;
  path.push_back(end);
  for (auto i = 0; i < pred_map.size(); i++){
    if (pred_map[temp] == start){
      break;
    }
    else{
      path.push_back(pred_map[temp]);
      temp = pred_map[temp];
    }
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());   // Reverse the vector of path to get path from start to end node.

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
  std::vector<std::string> path;
  std::string locname1 = FindClosestName(location1_name); // If user incorrectly spells the location, correct it by calling FindClosestName.
  std::string locname2 = FindClosestName(location2_name);
  std::string start = GetID(locname1);   // Get start node from location
  std::string end = GetID(locname2);     // Get end node from location
  std::unordered_map<std::string, double> dist; // Create unordered map to save shortest distance.   
  std::unordered_map<std::string, std::string> predecessor;  // Create unordered map to save predecessors.

  if (start == "" || end == ""){        // If passed start and end nodes are empty strings.
    path = {""};
  }
  std::unordered_map<std::string, Node>::iterator it;
  for (it = data.begin(); it != data.end(); it++){
    if (data[it->first].id == start){
      dist.insert(std::pair<std::string, double> (start, 0));
    }
    else{
      dist.insert(std::pair<std::string, double> (data[it->first].id, DBL_MAX));
    }
  }
  bool stop;
  for (auto i = 1; i < dist.size(); i++){
    stop = true;                                                  // Set bool flag to true.    
    for (auto &x : dist){
      for (auto &n : data[x.first].neighbors){
        if (dist[n] > x.second + CalculateDistance(x.first, n)){   // Check each neighbor node’s new distance is shorter than the previous one or not.
          dist[n] = x.second + CalculateDistance(x.first, n);      // Update shortest distance.
          predecessor[n] = x.first;                                // Update predecessor map.
          stop = false;
        }
      }
    }
    if (stop == true){                                             // If stop sign is true, we break, else we can traverse new data map again.
      break;
    }
  }
  std::string temp = end;
  path.push_back(end);

  // Build map from the predecessor map.
  for (auto i = 0; i < predecessor.size(); i++){
    if (predecessor[temp] == start){
      break;
    }
    else{
      path.push_back(predecessor[temp]);
      temp = predecessor[temp];
    }
  }
  path.push_back(start);
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
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::string start = location_ids[0];
  std::vector<std::string> current_path = {start};
  std::vector<std::string> min_path;
  double minDist = DBL_MAX;
  TSP_BruteForceHelper(start, start, 0.0, current_path, minDist, min_path, location_ids, records);
  records.first = minDist;
  records.second.push_back(min_path);
  return records;
}

void TrojanMap::TSP_BruteForceHelper(std::string start, std::string current_node, 
                                     double current_cost, std::vector<std::string> &current_path,
                                     double &minDist, std::vector<std::string> &min_path,
                                     std::vector<std::string> location_ids,
                                     std::pair<double, std::vector<std::vector<std::string>>> &records){
                                       if (current_path.size() == location_ids.size()){
                                         double final_cost = current_cost + CalculateDistance(current_node, start);
                                         if (final_cost < minDist){
                                           minDist = final_cost;
                                           min_path = current_path;
                                           min_path.push_back(location_ids[0]);
                                           records.first = minDist;
                                           records.second.push_back(min_path);
                                         }
                                         return;
                                       }
                                       for (int j = 0; j < location_ids.size(); j++){
                                         if (std::find(current_path.begin(), current_path.end(), location_ids[j]) != current_path.end()){
                                           continue;
                                         }
                                         current_path.push_back(location_ids[j]);
                                         TSP_BruteForceHelper(start, location_ids[j], current_cost + CalculateDistance(current_node, location_ids[j]), current_path, minDist, min_path, location_ids, records);
                                         current_path.pop_back();
                                       }
                                     }


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::string start = location_ids[0];
  std::vector<std::string> current_path = {start};
  std::vector<std::string> min_path;
  double minDist = DBL_MAX;
  TSP_BacktrackingHelper(start, start, 0.0, current_path, minDist, min_path, location_ids, records);
  records.first = minDist;
  records.second.push_back(min_path);
  return records;
}

void TrojanMap::TSP_BacktrackingHelper(std::string start, std::string current_node, 
                                     double current_cost, std::vector<std::string> &current_path,
                                     double &minDist, std::vector<std::string> &min_path,
                                     std::vector<std::string> location_ids,
                                     std::pair<double, std::vector<std::vector<std::string>>> &records){
                                       if (current_path.size() == location_ids.size()){
                                         double final_cost = current_cost + CalculateDistance(current_node, start);
                                         if (final_cost < minDist){
                                           minDist = final_cost;
                                           min_path = current_path;
                                           min_path.push_back(location_ids[0]);
                                           records.first = minDist;
                                           records.second.push_back(min_path);
                                         }
                                         return;
                                       }
                                       if (current_cost >= minDist){
                                         return;
                                       }


                                       for (int j = 0; j < location_ids.size(); j++){
                                         if (std::find(current_path.begin(), current_path.end(), location_ids[j]) != current_path.end()){
                                           continue;
                                         }
                                         current_path.push_back(location_ids[j]);
                                         TSP_BacktrackingHelper(start, location_ids[j], current_cost + CalculateDistance(current_node, location_ids[j]), current_path, minDist, min_path, location_ids, records);
                                         current_path.pop_back();
                                       }
                                     }

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::string> current_path;
  for(auto i: location_ids){
    current_path.push_back(i);
  }
  current_path.push_back(location_ids[0]);
  double newDist;
  std::vector<std::string> existing_path = current_path;
  double minDist;
  std::vector<std::string> new_route;

    bool change = true;
    while(change) {
      change = false;
    minDist = CalculatePathLength(existing_path);
    for (int i = 1; i <= location_ids.size()-2; i++) {
        for (int k = i + 1; k <= location_ids.size()-1; k++) {
            new_route = TSP2optSwap(existing_path, i, k);
            newDist = CalculatePathLength(new_route);
            if (newDist < minDist) {
                existing_path = new_route;
                minDist = newDist;
                records.second.push_back(new_route);
                records.first = minDist;
                change = true;
            }
        }
    }
}

  return records;
}

std::vector<std::string> TrojanMap::TSP2optSwap(std::vector<std::string> &existing_path, int i, int k){
  std::vector<std::string> new_route(existing_path.begin(), existing_path.begin()+i);
  std::vector<std::string> rev(existing_path.begin()+i, existing_path.begin()+k+1);
  std::reverse(rev.begin(),rev.end());
  new_route.insert(new_route.end(),rev.begin(),rev.end());
  new_route.insert(new_route.end(), existing_path.begin()+k+1, existing_path.end());
  return new_route; 
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
  std::fstream fin;
  
  fin.open(locations_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  
  while(getline(fin, line)){
    std::stringstream s(line);
    while(getline(s, word, ',')){
      location_names_from_csv.push_back(word);
    }
  }
  
  fin.close();

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
  std::fstream fin;
  
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  
  while(getline(fin, line)){
    std::stringstream st(line);
    std::string parent;
    std::string neighbor;
    std::vector<std::string> dependency;
    int c = 0;
    while(getline(st, word, ',')){
      if (c == 0){
        parent = word;
        dependency.push_back(parent);
      }
      else if (c == 1){
        neighbor = word;
        dependency.push_back(neighbor);
      }
      c++;
    }
    dependencies_from_csv.push_back(dependency);
  }
  
  fin.close();
  
  return dependencies_from_csv;
}

// Function to run DFS
void TrojanMap::DFSHelper(std::string loc, 
                          std::unordered_map<std::string, std::vector<std::string>> adj,
                          std::unordered_map<std::string, std::string> &visited,
                          std::vector<std::string> &result,
                          bool &flag){
                            visited[loc] = "being visited";

                            std::vector<std::string>::iterator it;
                            for (it = adj[loc].begin(); it != adj[loc].end(); it++){
                              if (visited[*it] == "not visited"){
                                DFSHelper(*it, adj, visited, result, flag);
                              }
                              else if (visited[*it] == "being visited"){
                                flag = true;
                                break;
                              }
                            }
                            if (flag == true){
                              result = {""};
                            }
                            else{
                              visited[loc] = "visited";
                              result.push_back(loc);
                            }
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
  std::unordered_map<std::string, std::vector<std::string>> adj;
  std::unordered_map<std::string, std::string> visited;
  bool flag = false;

  for (auto location : locations){
    std::vector<std::string> temp;
    adj[location] = temp;
    visited[location] = "not visited";
  }

  for (auto dependency : dependencies){
    adj[dependency[0]].push_back(dependency[1]);
  }

  DFSHelper(locations[0], adj, visited, result, flag);
  for (auto x: visited){
    if (x.second == "not visited"){
      DFSHelper(x.first, adj, visited, result, flag);
    }
  }
  std::reverse(result.begin(), result.end());

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
  auto lat1 = GetLat(id);
  auto lon1 = GetLon(id);
  if (lon1 >= square[0] && lon1 <= square[1] && lat1 <= square[2] && lat1 >= square[3] ){  // Condition to check if values lie in square.
    return true;
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
  for (auto it = data.begin(); it != data.end(); it++){
    if (inSquare(it->first, square)){      // If ids lie inside square push it to subgraph vector.
      subgraph.push_back(it->first);
    }
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

bool TrojanMap::CycleHelper(std::string current_id, 
                            std::unordered_map<std::string, bool> &visited,
                            std::string parent_id,
                            std::vector<double> &square,
                            std::unordered_map<std::string, std::vector<std::string>> &predecessors,
                            std::vector<std::string> &cycle_path){

                              visited[current_id] = true;

                              std::vector<std::string> neighbours = GetNeighborIDs(current_id);   // Get neighbors of current node.
                              predecessors.insert(std::pair<std::string,std::vector<std::string>> (parent_id,neighbours)); // Insert in predecessors map the parent id and its neighbors.
                              for(int i = 0; i < neighbours.size(); i++){    // Traverse through each neighbor of the node.
                                if (inSquare(neighbours[i], square)){
                                  if (!visited[neighbours[i]]){     // Condition to check if it is visited
                                    bool test = CycleHelper(neighbours[i], visited, current_id, square, predecessors, cycle_path);
                                    if(test){
                                      cycle_path.push_back(neighbours[i]);
                                      return true;
                                    }
                                  }
                                  else{
                                    if(neighbours[i]!=parent_id){    // If neighbor is visited and is not the parent, a back edge is found
                                      cycle_path.push_back(neighbours[i]);
                                      return true;
                                    }
                                  }
                                }
                              }
                              return false;
                            }

bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  std::vector<std::string> cycle_path;
  std::unordered_map<std::string,bool> visited;
  std::unordered_map<std::string,std::vector<std::string>> predecessors;
  for (auto s: subgraph){
    visited[s] = false;
  }
  for (auto &n : visited){
    if (n.second == false){
      if(CycleHelper(n.first, visited, "-1", square, predecessors, cycle_path)){
        return true;
      }
    }
  }
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
  typedef std::pair<std::string, double> pq;
  std::priority_queue<pq, std::vector<pq>, std::greater<pq>> minheap;
  std::string closest_name = FindClosestName(name);
  std::string name_id = GetID(closest_name);
  for (auto &x : data){
    for (auto &a : data[x.first].attributes){
      if (a == attributesName && x.first != name_id){
        double d = CalculateDistance(x.first, name_id);
        minheap.push(std::make_pair(x.first, d));
      }
    }
  }
  for (int j = 0; j < k; j++){
    if (minheap.empty()){
      break;
    }
    if (minheap.top().second > r){
      break;
    }
    if (minheap.top().second == 0){
      minheap.pop();
    }
    res.push_back(minheap.top().first);
    minheap.pop();
  }
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
