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
```
![1](https://github.com/user-attachments/assets/5681ae5d-20b0-444d-b9b6-e7f3a95152ea)

# Result
manager.FinalPathList-----save Inflection node 

manager.SmoothValType.SmoothFinalIndex -----save smallest cost index of manager.FinalPathList

you can judge PathFind like this to use smallest cost index

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
	if manager.PathFind(0, 0, 9, 9, false, true, false) {
		for _, val := range manager.SmoothValType.SmoothFinalIndex {
			fmt.Println("x:", manager.FinalPathList[val].X, "\ty:", manager.FinalPathList[val].Y)
		}
	}
 }
```
