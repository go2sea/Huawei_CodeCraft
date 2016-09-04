# Huawei_CodeCraft
###第二届华为软件经营挑战赛-寻路，京津东北赛区32强
####赛题：在一个加权有向图中，寻找经过指定点集的无环较短路径。<br/>
成绩：京津东北赛区32强。  

算法：改进的分枝限界和蚁群算法结合、改进的贪心搜索等算法。  

主要思想：区别于旅行商问题，赛题中节点重要性有差别。通过先期的分支限界、改进的贪心搜索等，对不同性质的节点设置不同的信息素初值。在后续蚁群算法中这个预处理效果明显。  

FZ.h：分支限界，定时清除队列中的非路口点
ONLY.h：同是分支限界，区别FZ，只有路口点影响信息素
DFS.h：对路口点的深度优先搜索