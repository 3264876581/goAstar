# BestAstar
a fast，smallest cost Astar search For moba，mmo game even for Autonomous driving and so on...
# Use
```go
func main() {
	var manager = DataMgr.NewMapManager(10, 10)
	//SetObstacle
	manager.SetObstacle(2, 2)
	manager.SetObstacle(3, 2)
	manager.SetObstacle(4, 2)
	manager.SetObstacle(2, 3)
	manager.SetObstacle(3, 3)
	manager.SetObstacle(4, 3)
	manager.SetObstacle(2, 4)
	manager.SetObstacle(3, 4)
	manager.SetObstacle(4, 4)
	//PathFind
	manager.PathFind(0, 0, 9, 9, true, true, true)
}
![6405b86a-7ea9-409e-90a2-18b6edf13f98](https://github.com/user-attachments/assets/13a60ff3-37cc-4698-acc4-57b6d1e039ed)
