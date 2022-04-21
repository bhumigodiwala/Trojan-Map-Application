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

