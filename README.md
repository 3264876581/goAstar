# BestAstar
a fast，smallest cost Astar search For moba，mmo game even for Autonomous driving and so on...
# Use
```go
func main() {
	var manager = DataMgr.NewMapManager(10, 10)
	if manager.PathFind(0, 0, 9, 9, true, false, false) {
		for _, val := range manager.SmoothValType.SmoothFinalIndex {
			fmt.Println("x:", manager.FinalPathList[val].X, "\ty:", manager.FinalPathList[val].Y)
		}
	}
}
