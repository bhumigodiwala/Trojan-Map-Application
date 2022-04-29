#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"
#include <vector>
#include <unordered_set>


// Phase 1
// Test Autocomplete function
TEST(TrojanMapTest, AutocompleteTest) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("USC");
  std::unordered_set<std::string> gt = {"USC Roski Eye Institute","USC Parking","USC Village Gym","USC Fisher Museum of Art","USC Credit Union"}; // groundtruth for "USC"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("usc");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("usC"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("USC"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  
  // Test Bank of America
  auto position = m.GetPosition("Bank of America");
  std::pair<double, double> gt1(34.0251870, -118.2841713); // groundtruth for "Bank of America"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Ralphs");
  std::pair<double, double> gt2(34.0317653, -118.2908339); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);

  // Test Chipotle
  position = m.GetPosition("Chipotle");
  std::pair<double, double> gt(34.0169985,-118.2822768); // groundtruth for "Chipotle"
  EXPECT_EQ(position, gt);
  
  // Test CVS Pharmacy
  position = m.GetPosition("CVS Pharmacy");
  std::pair<double, double> gt3(34.0234847, -118.2793109); // groundtruth for "CVS Pharmacy"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("trojan", "troy"), 3);
  EXPECT_EQ(m.CalculateEditDistance("ralphs", "rolph"), 2);
}

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  std::string o1 = "Starbucks";
  // std::transform(o1.begin(), o1.end(), o1.begin(), ::tolower);
  EXPECT_EQ(m.FindClosestName("STARBUcks"), o1);
  
  std::string o2 = "Trader Joes";
  // std::transform(o2.begin(), o2.end(), o2.begin(), ::tolower);
  EXPECT_EQ(m.FindClosestName("TRader Jaes"), o2);

  std::string o3 = "Tommy Trojan";
  // std::transform(o3.begin(), o3.end(), o3.begin(), ::tolower);
  EXPECT_EQ(m.FindClosestName("Tmmy Trjn"), o3);

}


// Phase 2
// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698","4015377690","4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229","122719216","6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650"}; // Expected path
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




  auto path1 = m.CalculateShortestPath_Dijkstra("Ralphs", "CVS Pharmacy");
  std::vector<std::string> gt1{
      "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","3398621886","3398621887","6816193695","5690152756","6804883324","3398621888","6813416123","6813416160","6818390184","6879730013","6879730020","6879730019","6818390183","6813405228","6813405233","3398578896","6813379394","7071032400","6813379424","3398578901","4399698012","3398582607","4399698007","3398582608","4399698004","6813379520","9591449482","6807200383","6813379504","3398582609","3443323842","6813379503","6045067410","6813565328","3088548446"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path1) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt1) << "miles" << std::endl;
  EXPECT_EQ(path1, gt1);
  
  // Reverse the input from Ralphs to Target
  path1 = m.CalculateShortestPath_Dijkstra("CVS Pharmacy", "Ralphs");
  std::reverse(gt1.begin(),gt1.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path1) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt1) << "miles" << std::endl;
  EXPECT_EQ(path1, gt1);





  auto path2 = m.CalculateShortestPath_Dijkstra("Trader Joes", "Tommy Trojan");
  std::vector<std::string> gt2{
      "5237417649","9591449485","6396649383","6814769289","6813379584","6813360961","6813379480","6813360960","6814620882","6813360954","6813360952","6813379420","6813360951","6813360936","6813379467","6813379466","6813379465","6813379464","3402887075","6813379432","4536989637","4536989640","3443310465","6813379491","6818390136","3433701978","4536989636","3432332948","2776870273","9559677954","6814620870","122844995","3229367545","3229367546","4399697341","4399697358","2305853438"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path2) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt2) << "miles" << std::endl;
  EXPECT_EQ(path2, gt2);
  
  // Reverse the input from Ralphs to Target
  path2 = m.CalculateShortestPath_Dijkstra("Tommy Trojan", "Trader Joes");
  std::reverse(gt2.begin(),gt2.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path2) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt2) << "miles" << std::endl;
  EXPECT_EQ(path2, gt2);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698","4015377690","4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229","122719216","6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Target
  path = m.CalculateShortestPath_Bellman_Ford("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);




  auto path1 = m.CalculateShortestPath_Bellman_Ford("Ralphs", "CVS Pharmacy");
  std::vector<std::string> gt1{
      "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","3398621886","3398621887","6816193695","5690152756","6804883324","3398621888","6813416123","6813416160","6818390184","6879730013","6879730020","6879730019","6818390183","6813405228","6813405233","3398578896","6813379394","7071032400","6813379424","3398578901","4399698012","3398582607","4399698007","3398582608","4399698004","6813379520","9591449482","6807200383","6813379504","3398582609","3443323842","6813379503","6045067410","6813565328","3088548446"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path1) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt1) << "miles" << std::endl;
  EXPECT_EQ(path1, gt1);
  
  // Reverse the input from Ralphs to Target
  path1 = m.CalculateShortestPath_Bellman_Ford("CVS Pharmacy", "Ralphs");
  std::reverse(gt1.begin(),gt1.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path1) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt1) << "miles" << std::endl;
  EXPECT_EQ(path1, gt1);





  auto path2 = m.CalculateShortestPath_Bellman_Ford("Trader Joes", "Tommy Trojan");
  std::vector<std::string> gt2{
      "5237417649","9591449485","6396649383","6814769289","6813379584","6813360961","6813379480","6813360960","6814620882","6813360954","6813360952","6813379420","6813360951","6813360936","6813379467","6813379466","6813379465","6813379464","3402887075","6813379432","4536989637","4536989640","3443310465","6813379491","6818390136","3433701978","4536989636","3432332948","2776870273","9559677954","6814620870","122844995","3229367545","3229367546","4399697341","4399697358","2305853438"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path2) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt2) << "miles" << std::endl;
  EXPECT_EQ(path2, gt2);
  
  // Reverse the input from Ralphs to Target
  path2 = m.CalculateShortestPath_Bellman_Ford("Tommy Trojan", "Trader Joes");
  std::reverse(gt2.begin(),gt2.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path2) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt2) << "miles" << std::endl;
  EXPECT_EQ(path2, gt2);
}


// Test cycle detection function
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.288, -118.250, 34.035, 34.005};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.320, -118.318, 34.040, 34.039};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);

  // Test case 3
  std::vector<double> square3 = {-118.250, -118.235, 34.025, 34.000};
  auto sub3 = m.GetSubgraph(square3);
  bool result3 = m.CycleDetection(sub3, square3);
  EXPECT_EQ(result3, true);
}

// Test topological sort function
TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
  
  std::vector<std::string> location_names1 = {"Trader Joes","Starbucks","LATTC/Ortho Institute"};
  std::vector<std::vector<std::string>> dependencies1 = {{"Trader Joes","Starbucks"}, {"Trader Joes","LATTC/Ortho Institute"}, {"Starbucks","LATTC/Ortho Institute"}};
  auto result1 = m.DeliveringTrojan(location_names1, dependencies1);
  std::vector<std::string> gt1 ={"Trader Joes","Starbucks","LATTC/Ortho Institute"};
  EXPECT_EQ(result1, gt1);

  std::vector<std::string> location_names2 = {"Ralphs","Starbucks","LATTC/Ortho Institute"};
  std::vector<std::vector<std::string>> dependencies2 = {{"Ralphs","Starbucks"}, {"Ralphs","LATTC/Ortho Institute"}, {"Starbucks","LATTC/Ortho Institute"}};
  auto result2 = m.DeliveringTrojan(location_names2, dependencies2);
  std::vector<std::string> gt2 ={"Ralphs","Starbucks","LATTC/Ortho Institute"};
  EXPECT_EQ(result2, gt2);

  std::vector<std::string> location_names3 = {"Trader Joes","Starbucks","Five Guys"};
  std::vector<std::vector<std::string>> dependencies3 = {{"Trader Joes","Starbucks"}, {"Trader Joes","Five Guys"}, {"Starbucks","Five Guys"}};
  auto result3 = m.DeliveringTrojan(location_names3, dependencies3);
  std::vector<std::string> gt3 ={"Trader Joes","Starbucks","Five Guys"};
  EXPECT_EQ(result3, gt3);
}


// Phase 3
// Test TSP function

//Brute Force
TEST(TrojanMapTest, TSP1) {
  TrojanMap m;
  
  std::vector<std::string> input1{"8566227755","6276439468","6788498757","7197964033","6805221420","123656718","6807927346","7197964012"}; // Input location ids 
  auto result1 = m.TravellingTrojan_Brute_force(input1);
  std::cout << "My path length: "  << result1.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt1{"8566227755","6276439468","6788498757","6805221420","7197964033","7197964012","6807927346","123656718","8566227755"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt1) << "miles" << std::endl; // Print the gt path lengths
  bool flag1 = false;
  if (gt1 == result1.second[result1.second.size()-1]) // clockwise
    flag1 = true;
  std::reverse(gt1.begin(),gt1.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt1 == result1.second[result1.second.size()-1]) 
    flag1 = true;
  
  EXPECT_EQ(flag1, true);





  std::vector<std::string> input2{"1836112325","2871078026","1865156358","6808311401","6817165995"}; // Input location ids 
  auto result2 = m.TravellingTrojan_Brute_force(input2);
  std::cout << "My path length: "  << result2.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt2{"1836112325","1865156358","2871078026","6808311401","6817165995","1836112325"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt2) << "miles" << std::endl; // Print the gt path lengths
  bool flag2 = false;
  if (gt2 == result2.second[result2.second.size()-1]) // clockwise
    flag2 = true;
  std::reverse(gt2.begin(),gt2.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt2 == result2.second[result2.second.size()-1]) 
    flag2 = true;
  
  EXPECT_EQ(flag2, true);





  std::vector<std::string> input3{"1855173106","4096366481","2193435046","63043854","8179059377","122607557","1838283858","7432340645","6813405249","123338991"}; // Input location ids 
  auto result3 = m.TravellingTrojan_Brute_force(input3);
  std::cout << "My path length: "  << result3.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt3{"1855173106","7432340645","63043854","123338991","6813405249","2193435046","8179059377","122607557","1838283858","4096366481","1855173106"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt3) << "miles" << std::endl; // Print the gt path lengths
  bool flag3 = false;
  if (gt3 == result3.second[result3.second.size()-1]) // clockwise
    flag3 = true;
  std::reverse(gt3.begin(),gt3.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt3 == result3.second[result3.second.size()-1]) 
    flag3 = true;
  
  EXPECT_EQ(flag3, true);
}

//BackTracking
TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  
  std::vector<std::string> input1{"8566227755","6276439468","6788498757","7197964033","6805221420","123656718","6807927346","7197964012"}; // Input location ids 
  auto result1 = m.TravellingTrojan_Backtracking(input1);
  std::cout << "My path length: "  << result1.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt1{"8566227755","6276439468","6788498757","6805221420","7197964033","7197964012","6807927346","123656718","8566227755"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt1) << "miles" << std::endl; // Print the gt path lengths
  bool flag1 = false;
  if (gt1 == result1.second[result1.second.size()-1]) // clockwise
    flag1 = true;
  std::reverse(gt1.begin(),gt1.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt1 == result1.second[result1.second.size()-1]) 
    flag1 = true;
  
  EXPECT_EQ(flag1, true);



  std::vector<std::string> input2{"1836112325","2871078026","1865156358","6808311401","6817165995"}; // Input location ids 
  auto result2 = m.TravellingTrojan_Backtracking(input2);
  std::cout << "My path length: "  << result2.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt2{"1836112325","1865156358","2871078026","6808311401","6817165995","1836112325"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt2) << "miles" << std::endl; // Print the gt path lengths
  bool flag2 = false;
  if (gt2 == result2.second[result2.second.size()-1]) // clockwise
    flag2 = true;
  std::reverse(gt2.begin(),gt2.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt2 == result2.second[result2.second.size()-1]) 
    flag2 = true;
  
  EXPECT_EQ(flag2, true);




  std::vector<std::string> input3{"1855173106","4096366481","2193435046","63043854","8179059377","122607557","1838283858","7432340645","6813405249","123338991"}; // Input location ids 
  auto result3 = m.TravellingTrojan_Backtracking(input3);
  std::cout << "My path length: "  << result3.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt3{"1855173106","7432340645","63043854","123338991","6813405249","2193435046","8179059377","122607557","1838283858","4096366481","1855173106"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt3) << "miles" << std::endl; // Print the gt path lengths
  bool flag3 = false;
  if (gt3 == result3.second[result3.second.size()-1]) // clockwise
    flag3 = true;
  std::reverse(gt3.begin(),gt3.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt3 == result3.second[result3.second.size()-1]) 
    flag3 = true;
  
  EXPECT_EQ(flag3, true);
}

//2-OPT
TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  
  std::vector<std::string> input1{"8566227755","6276439468","6788498757","7197964033","6805221420","123656718","6807927346","7197964012"}; // Input location ids 
  auto result1 = m.TravellingTrojan_2opt(input1);
  std::cout << "My path length: "  << result1.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt1{"8566227755","6276439468","6788498757","6805221420","7197964033","7197964012","6807927346","123656718","8566227755"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt1) << "miles" << std::endl; // Print the gt path lengths
  bool flag1 = false;
  if (gt1 == result1.second[result1.second.size()-1]) // clockwise
    flag1 = true;
  std::reverse(gt1.begin(),gt1.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt1 == result1.second[result1.second.size()-1]) 
    flag1 = true;
  
  EXPECT_EQ(flag1, true);




  std::vector<std::string> input2{"1836112325","2871078026","1865156358","6808311401","6817165995"}; // Input location ids 
  auto result2 = m.TravellingTrojan_2opt(input2);
  std::cout << "My path length: "  << result2.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt2{"1836112325","1865156358","2871078026","6808311401","6817165995","1836112325"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt2) << "miles" << std::endl; // Print the gt path lengths
  bool flag2 = false;
  if (gt2 == result2.second[result2.second.size()-1]) // clockwise
    flag2 = true;
  std::reverse(gt2.begin(),gt2.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt2 == result2.second[result2.second.size()-1]) 
    flag2 = true;
  
  EXPECT_EQ(flag2, true);




  std::vector<std::string> input3{"1855173106","4096366481","2193435046","63043854","8179059377","122607557","1838283858","7432340645","6813405249","123338991"}; // Input location ids 
  auto result3 = m.TravellingTrojan_2opt(input3);
  std::cout << "My path length: "  << result3.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt3{"1855173106","7432340645","63043854","123338991","6813405249","2193435046","8179059377","122607557","1838283858","4096366481","1855173106"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt3) << "miles" << std::endl; // Print the gt path lengths
  bool flag3 = false;
  if (gt3 == result3.second[result3.second.size()-1]) // clockwise
    flag3 = true;
  std::reverse(gt3.begin(),gt3.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt3 == result3.second[result3.second.size()-1]) 
    flag3 = true;
  
  EXPECT_EQ(flag3, true);
}


// Test FindNearby points
TEST(TrojanMapTest, FindNearby) {
  TrojanMap m;
  
  auto result1 = m.FindNearby("bank", "Bank of America", 10, 10);
  std::vector<std::string> ans1{"9591449441", "9591449465"};
  EXPECT_EQ(result1, ans1);

  auto result2 = m.FindNearby("fast_food", "Cava", 10, 10);
  std::vector<std::string> ans2{"1759017528", "1759017530", "1759017531", "3088547686", "3577173161", "4547476733", "4577908517", "4927493958", "5567724155", "5567733799"};
  EXPECT_EQ(result2, ans2);

  auto result3 = m.FindNearby("hotel", "USC Hotel", 10, 10);
  std::vector<std::string> ans3{"9051883397"};
  EXPECT_EQ(result3, ans3);
}
