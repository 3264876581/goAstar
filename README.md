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
	manager.PathFind(0, 0, 9, 9, true, true, true)
}
