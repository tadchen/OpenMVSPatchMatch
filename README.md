# openMVS稠密重建

## 1 读入空三数据

### 1.1 Load colmap.m3a数据
DensiPointCloud.cpp-int main(int argc, LPCTSTR *argv)
Scene.cpp-scene.Load(OPT::strInputFileName)

- bool Scene::LoadInterface(const String & fileName)

	- 读入Scene::PlatformArr,包含平台上相机相对平台位姿RC(一般是I和0矩阵)，相机内参K（正则化）

	- 读入Scene::ImageArr,包含图像ID,图像名称name,位姿IDposeID,platformID,cameraID，图像宽高（原始尺寸）,image::Camera影像位姿K,R,C,P

		- 获取image::Camera,该影像对应的位姿K,R,C,P
void Image::UpdateCamera(const PlatformArr& platforms)--将R_p2c,R_w2p转到R_w2c
Camera Image::GetCamera(const PlatformArr& platforms, const Image8U::Size& resolution) const

	- 读入Scene::PointCloud，包含所有稀疏点位置，颜色，以及每个点可视的图像id数组PointCloud::ViewArr,以及每个点可视的图像id的权重数组PointCloud::WeightArr（当前是0）

### 数据文件准备：
1.更新图像数据位置，Scene::ImageArr中图像的name会更新为comap.m3a文件夹下的images目录下的图像名（InterfaceColamp会复制产生images文件夹）
void UpdateImagesPath(Scene &scene, const std::filesystem::path &workFolderName)
2.读取activa-images.txt(在colmap.m3a同层目录，InterfaceColmap产出）
3.创建Detail目录(在colmap.m3a同层目录）
4.读取boud-box.txt(在colmap.m3a同层目录，可为空代表没有限制区域，scene.obb为0)

## 稠密点云重建
bool Scene::DenseReconstruction(int nFusionMode, bool bCrop2ROI, float fBorderROI)

### PatchMatch算法进行稠密重建 if nFusionMode==0

- 计算深度图
bool Scene::ComputeDepthMaps(DenseDepthMapData& data)

	- 1 视图选择

		- DenseDepthMapData

			- Scene& scene; 场景引用

			- IIndexArr images; 处理的图像数组

			- IIndexArr neighborsMap; 图像数组对应的最佳邻居ID

			- DepthMapsData depthMaps;

				- Scene& scene; 场景引用

				- DepthDataArr arrDepthData; 所有的深度图数据

					- ViewDataArr images;  用于计算深度图的图像数组

					- ViewScoreArr neighbors; 所有可见该深度图的视图得分（降序）

					- IndexArr points; 该深度图相关的稀疏点序列

					- DepthMap depthMap; 深度图

					- NormalMap normalMap; 法线图

					- ConfidenceMap confMap;  置信度图

					- float dMin, dMax; 最小最大深度

		- 0 图像数据准备

			- 1) 缩放Scene::images 
根据输入参数OPTDENSE::nResolutionLevel进行图像缩放以及内参缩放（Scene::ImageArr中Image::Camera）

			- 
2)DenseDepthMapData& data, data.images写入Scene::images中图像ID,局部变量IIndexArr imagesMap;记录全局图像ID和data.images真实有效ID对应关系（以防存在Scene::images无效情况）

		- 1 图像对选择打分

			- 1）计算所有视图及其邻居视图分数
for 遍历data.images
    计算当前视角的所有邻居视图分数，存储于data.depthMaps.arrDepthData数组的DepthData& depthData中

				- bool DepthMapsData::SelectViews(DepthData& depthData);

					- bool Scene::SelectNeighborViews(
    uint32_t ID, IndexArr &points, unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, unsigned nInsideROI)
对当前视角选取最佳匹配对

						- for 遍历所有稀疏点
    0）计算当前视角总和深度：计算该稀疏点在当前视角深度depth，if depth>0则添加当前视角ID进IndexArr &points；当前影像平均深度加和imageData.avgDepth += depth;点数加1(为计算平均深度)
    for 遍历稀疏点所有邻居视图pointcloud.pointViews
        计算该稀疏点在两视角下：
        1）计算两视角角度权重w_theta,累计角度sumAngle
        2）计算两视角深度比权重w_scale,累计深度比例sumScale
        3）计算两视角当前分数score=w_theta*w_scale,累计分数sumScore
    done
done
4) 计算当前视角平均深度
for 遍历所有邻居视角Scene::ImageArr
    for 遍历所有稀疏点
        将所有有效稀疏点投影到当前视角像素A和邻居视角像素B，保留AB都有效内容。
        5）计算两视角共视点集的area占比
    6) 计算两视图score
    done
done

							- 1 两视图角度权重

								-  

									-  

							- 2 两视图深度比例权重

								-  

									-  

							- 3 两视角共视点集area占比

								- 将refrence视角的图像化成16*16的格子，按投影点算像素的占有率，这个比例即当做neighbor视角与reference视角的overlap值w_area

								-  计算投影点集的凸包，计算凸包面积与图像面积的比值作为overlap值w_area

									- 凸包面积算法：鞋带算法（叉积、有向面积）

							- 4 计算两视图score

								-  

									- 视角的邻居信息更新在Scene::ImageArr中的Image::ViewScoreArr neighbors中。每个ViewScore记录了邻居ID,共视稀疏点ID,两视图平均角度avgAngle,两视图平均深度比例avgScale,两视图共视面积area,两视图分数score

							- 5 当前视角的所有邻居按分数进行降序排序

					- static bool FilterNeighborViews(ViewScoreArr &neighbors...)
过滤掉不符合指定范围的邻居视图

						- 两视图共视占比area小于3/4 or
两视图平均深度比例avgScale不在[0.2,2.4] or
两视图平均角度avgAngle不在[3,45]度
满足上述的两视图关系会被清除

			- 2）计算全局视图选择
如果设置了OPTDENSE::nNumViews==1

				- bool DepthMapsData::SelectViews(IIndexArr& images, IIndexArr& imagesMap, IIndexArr& neighborsMap)
目标是全局选择每个图像的最佳目标视图，尝试同时选取图像对覆盖整个场景；
顶点是视图，如果两个视图互为邻居，则两个顶点由一条边连接。避免两视图互相为最佳视图情形。
全局视图选择问题转换为马尔科夫随机场MRF的优化问题。

					- 1 构建马尔科夫随机场

						- 添加节点和边

							- 节点:每个image视为一个节点
为每个节点添加一元成本

								- 一元成本：和图像对得分成反比
arrUnary一元成本数组，size是neighbors.size()+1(当前节点邻居数+1)

									- avgScore是所有图像对的分数的均值。y(xi)=N代表xi节点没有选中邻居作为最优邻居。如果xi有邻居图像对却没有选择视图连接，成本会乘以Cnone=6.0放大惩罚。r_none = 0.01(对本身没有邻居的视图不做大惩罚)。最终的小成本原则是i有邻居视图时应选择一个邻居视图作为最优视图且选择的视图的得分应尽可能高

							- 边：节点选择某视图为最优视图即以边连接

								- 二元图像对成本：视角i和视角j本身覆盖的面积area与视角i和视角j实际选择的最佳邻居后实际覆盖的面积之比
arrPairwise二元成本数组，size是(neighbours_I.size() + 1)*(neighbours_J.size() + 1)

									- 当视角i和j互相选为最佳视角后惩罚项是Cpenalty = 7.2(较大的惩罚值)；r = 0.3；当视角i或j其中一个视角没有选择其他视角时惩罚为Cempty = 2.4；最终的小成本原则是i，j都有选择最佳视角，但不互相为最佳视角，且选择的最佳视角应尽量满足areaI和areaj比areaij大

					- 2 最小化能量函数

						- 采用TRW-S算法​​Sequential Tree-Reweighted Message Passing​​（顺序树重加权消息传递算法）

							- TRW-S的核心思想是​​将复杂的图结构分解为多个树结构的组合​​。它通过一个巧妙的“重加权”过程，将原始的MRF能量函数表示为一系列树结构上能量函数的凸组合。由于树结构上的推理是精确且高效的，算法随后在这些树上并行地进行消息传递（计算最小能量），并通过协调不同树的解，逐步逼近整个图的最优解。其中“顺序”一词体现了其消息传递遵循特定顺序，有助于改善收敛性

					- 3 最终结果存储在DepthData data的DenseDepthMapData::IIndexArr 的data.images和data.neighborsMap中

						- data.images: 0,1,3...
data.neighboursMap:4,14,43,...

	- 2 深度图初始化

		- 1 初始化cuda设备
CUresult initDevice(int deviceID=-1);
选择GFLOPS最高的设备作为运行设备

### SGM算法 if nFusionMode==-1

