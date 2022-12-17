#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapTest, Autocomplete1) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Sub");
  std::unordered_set<std::string> gt = {"Subway", "Subway 1"};
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("sub");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case
  names = m.Autocomplete("sUB");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case
  names = m.Autocomplete("SUB");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// Phase 1
// Test Autocomplete function
TEST(TrojanMapTest, Autocomplete2) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Sub");
  std::unordered_set<std::string> gt = {"Subway", "Subway 1"};
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("sub");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case
  names = m.Autocomplete("sUB");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case
  names = m.Autocomplete("SUB");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}


TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
 
  // Test Chick-fil-A
  auto position = m.GetPosition("Chick-fil-A");
  std::pair<double, double> gt1(34.0167334, -118.2825307); // groundtruth for "Chick-fil-A"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Ralphs");
  std::pair<double, double> gt2(34.0317653, -118.2908339); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = m.GetPosition("Target");
  std::pair<double, double> gt3(34.0257016, -118.2843512); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}


TEST(TrojanMapTest, CalculateEditDistance1) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("Ralphs", "Rolphs"), 1);
  EXPECT_EQ(m.CalculateEditDistance("Target", "Chipotle"), 7);
}

TEST(TrojanMapTest, CalculateEditDistance2) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("", ""), 0);
  EXPECT_EQ(m.CalculateEditDistance("", "Subway"), 6);
}

TEST(TrojanMapTest, CalculateEditDistance3) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("pci grnd 1", "Pico & Grand 1"), 5);
  EXPECT_EQ(m.CalculateEditDistance("Target", ""), 6);
}

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName1) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Subday"), "Subway");
}

TEST(TrojanMapTest, FindClosestName2) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Chipptle"), "Chipotle");
}

TEST(TrojanMapTest, FindClosestName3) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Tarrgt"), "Target");
}

// Phase 2
// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
 
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
      "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
      "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
      "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
      "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
      "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
      "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
      "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
      "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
      "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
      "6814916515","6820935910","4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
 
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra1) {
  TrojanMap m;

  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040158","4380040167","6805802087",
      "8410938469","6813416131","7645318201","6813416130","6813416129",
      "123318563","452688940","6816193777","123408705","6816193774","452688933",
      "452688931","123230412","6816193770","6787470576","4015442011","6816193692",
      "6816193693","6816193694","4015377691","544693739","6816193696","6804883323",
      "6807937309","6807937306","6816193698","4015377690","4015377689","122814447",
      "6813416159","6813405266","4015372488","4015372487","6813405229","122719216",
      "6813405232","4015372486","7071032399","4015372485","6813379479","6813379584",
      "6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
 
  // Reverse the input from Ralphs to Target
  path = m.CalculateShortestPath_Dijkstra("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;

  // Test from Chase to Chipotle
  auto path = m.CalculateShortestPath_Dijkstra("Chass", "Chiptle");
  std::vector<std::string> gt{
      "9591449441","9559739232","3398574883","6813379494","6813379495",
      "6813379544","6813379545","6813379536","6813379546","6813379547",
      "6814916522","6814916523","1732243620","4015372469","4015372463",
      "6819179749","1732243544","6813405275","348121996","348121864",
      "6813405280","1472141024","6813411590","216155217","6820935908",
      "9446678100","732641023"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
 
  // Reverse the input from Chase to Chipotle
  path = m.CalculateShortestPath_Dijkstra("Chiptle", "Chass");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
 
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
      "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
      "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
      "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
      "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
      "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
      "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
      "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
      "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
      "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
      "6814916515","6820935910","4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
 
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford1) {
  TrojanMap m;
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040158","4380040167","6805802087",
      "8410938469","6813416131","7645318201","6813416130","6813416129",
      "123318563","452688940","6816193777","123408705","6816193774","452688933",
      "452688931","123230412","6816193770","6787470576","4015442011","6816193692",
      "6816193693","6816193694","4015377691","544693739","6816193696","6804883323",
      "6807937309","6807937306","6816193698","4015377690","4015377689","122814447",
      "6813416159","6813405266","4015372488","4015372487","6813405229","122719216",
      "6813405232","4015372486","7071032399","4015372485","6813379479","6813379584",
      "6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
 
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford2) {
  TrojanMap m;
  // Test from Chase to Chipotle
  auto path = m.CalculateShortestPath_Bellman_Ford("Chass", "Chiptle");
  std::vector<std::string> gt{
      "9591449441","9559739232","3398574883","6813379494","6813379495",
      "6813379544","6813379545","6813379536","6813379546","6813379547",
      "6814916522","6814916523","1732243620","4015372469","4015372463",
      "6819179749","1732243544","6813405275","348121996","348121864",
      "6813405280","1472141024","6813411590","216155217","6820935908",
      "9446678100","732641023"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
 
  // Reverse the input from Chase to Chipotle
  path = m.CalculateShortestPath_Bellman_Ford("Chiptle", "Chass");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test cycle detection function
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
 
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);

  // Test case 3
  std::vector<double> square3 = {-118.220, -118.189, 34.040, 34.010};
  auto sub3 = m.GetSubgraph(square3);
  bool result3 = m.CycleDetection(sub3, square3);
  EXPECT_EQ(result3, false);
}



// Test cycle detection function
TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
 
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort2) {
  TrojanMap m;
 
  std::vector<std::string> location_names = {"1", "2", "3", "4", "5"};
  std::vector<std::vector<std::string>> dependencies = {{"1","2"}, {"1","3"}, {"4","3"}, {"4", "5"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"4", "5", "1", "3", "2"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort3) {
  TrojanMap m;
 
  std::vector<std::string> location_names = {"1", "2", "3"};
  std::vector<std::vector<std::string>> dependencies = {{"1", "2"}, {"2", "3"}, {"3", "1"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={""};
  EXPECT_EQ(result, gt);
}

// Phase 3
// Test TSP function
TEST(TrojanMapTest, TSPBruteForce1) {
  TrojanMap m;
 
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}


TEST(TrojanMapTest, TSPBruteForce2) {
  TrojanMap m;
 
  std::vector<std::string> input{"7875114135","6808227837","6818390175","6817078335","6816193750","7863630712","1716461476","7864199492"}; // Input location ids
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"7875114135","6808227837","6816193750","6817078335","7863630712","6818390175","1716461476","7864199492","7875114135"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSPBruteForce3) {
  TrojanMap m;
 
  std::vector<std::string> input{"6816889465","7867291941","7735888674","1932939403","7130092325","611590750","6787728629","7863689385","304904245","4015405543"}; // Input location ids
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6816889465","1932939403","4015405543","6787728629","7735888674","7863689385","7130092325","304904245","611590750","7867291941","6816889465"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSPBacktracking1) {
  TrojanMap m;
 
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSPBacktracking2) {
  TrojanMap m;
 
  std::vector<std::string> input{"7875114135","6808227837","6818390175","6817078335","6816193750","7863630712","1716461476","7864199492"}; // Input location ids
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"7875114135","6808227837","6816193750","6817078335","7863630712","6818390175","1716461476","7864199492","7875114135"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSPBacktracking3) {
  TrojanMap m;
 
  std::vector<std::string> input{"6816889465","7867291941","7735888674","1932939403","7130092325","611590750","6787728629","7863689385","304904245","4015405543"}; // Input location ids
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6816889465","1932939403","4015405543","6787728629","7735888674","7863689385","7130092325","304904245","611590750","7867291941","6816889465"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}


TEST(TrojanMapTest, TSP2opt1) {
  TrojanMap m;
 
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2opt2) {
  TrojanMap m;
 
  std::vector<std::string> input{"7875114135","6808227837","6818390175","6817078335","6816193750","7863630712","1716461476","7864199492"}; // Input location ids
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"7875114135","6808227837","6816193750","6817078335","7863630712","6818390175","1716461476","7864199492","7875114135"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2opt3) {
  TrojanMap m;
 
  std::vector<std::string> input{"6816889465","7867291941","7735888674","1932939403","7130092325","611590750","6787728629","7863689385","304904245","4015405543"}; // Input location ids
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6816889465","1932939403","4015405543","6787728629","7735888674","7863689385","7130092325","304904245","611590750","7867291941","6816889465"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1])
    flag = true;
 
  EXPECT_EQ(flag, true);
}

// Test FindNearby points
TEST(TrojanMapTest, FindNearby1) {
  TrojanMap m;
 
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 10);
  std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby2) {
  TrojanMap m;
 
  auto result = m.FindNearby("bar", "Rock & Reilly's", 10, 10);
  std::vector<std::string> ans{"5567714035", "6045038065"};
  EXPECT_EQ(result, ans);
}


TEST(TrojanMapTest, FindNearby3) {
  TrojanMap m;
 
  auto result = m.FindNearby("restaurant", "Pizza King", 10, 10);
  std::vector<std::string> ans{"2643391587", "4141781780", "4630579203", "5231970324", "5231970325", "5237417652", "5237417654", "5261316289", "5309228560", "5567727178",
};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby4) {
  TrojanMap m;
 
  auto result = m.FindNearby("gallery", "Track 16 Gallery", 10, 10);
  std::vector<std::string> ans;
  EXPECT_EQ(result, ans);
}
