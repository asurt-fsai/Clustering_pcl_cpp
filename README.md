# clustering - 3d lidar with pcl


앞으로 수정해야하고 고쳐나가야 할 부분:
<ol>
  <li>장애물 정확히 인식하기. 회전 시 장애물을 어떻게 판단할지 생각해보기</li>
  <li>장애물과의 거리 계산하는 코드 작성</li>
  <li>동적장애물 판단 후 정지</li>
  <li>정적장애물 회피 알고리즘 구현, 마지막 장애물을 회피하고, 본래 차선을 유지하도록 하기</li>
  <li>오른쪽, 왼쪽 장애물 둘 중 어느 것이 와도 장애물을 피하도록 하기</li>
  <li>차선 두 개중 진행차선이 아닌 옆 차선에 장애물이 있을 경우 장애물을 무시하고, 진행하도록 하기</li>
  <li>Imu 사용법 익히기</li>
  <li>SLAM</li>
</ol>
 
-------------------------
<h2> pcl::VoxelGrid<pcl::PointXYZ> vg </h2>

VoxelGrid는 Point Cloud위에 3차원 복셀 격자를 조합함. Voxel은 비유하면 Pixel의 3차원 버전이다.
vg 객체를 만들어 PointCloud를 Voxel Grid의 개수만큼 줄여 DownSampling을 합니다. 점이 많을 때 일정 간격으로 균일하게 점을 줄여 유용합니다.

-------------------------
<h2> pcl::PassThrough<pcl::PointXYZ> pass </h2>

x,y,z평면에 각각에 대해서 지정된 영역 밖의 점들은 모두 제거합니다. 
원하는 구역만 볼 수 있게하여 지나친 장애물이나 바닥을 제거하는 효과를 얻습니다.

-------------------------
<h2> pcl::EculideanClusterExtraction<pcl::PointXYZ> ec </h2>
setClusterTolerance, setMinClusterSize, SetMaxClusterSize를 수정하는 경우가 많았고,
Min은 Cluster로 인식할 수 있는 최소 점의 개수, Max는 Cluster로 인식할 수 있는 최대 점의 개수

  
참고 사이트:
<li>https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction</li>
<li>http://docs.ros.org/en/hydro/api/pcl/html/namespacepcl.html # pcl에서 모르는 함수 생기면 이 사이트에서 찾아보기
