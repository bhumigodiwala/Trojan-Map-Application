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
  std::transform(o1.begin(), o1.end(), o1.begin(), ::tolower);
  EXPECT_EQ(m.FindClosestName("STARBUcks"), o1);
  
  std::string o2 = "Trader Joes";
  std::transform(o2.begin(), o2.end(), o2.begin(), ::tolower);
  EXPECT_EQ(m.FindClosestName("TRader Jaes"), o2);

  std::string o3 = "Tommy Trojan";
  std::transform(o3.begin(), o3.end(), o3.begin(), ::tolower);
  EXPECT_EQ(m.FindClosestName("Tmmy Trjn"), o3);

}


