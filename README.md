# BestAstar(Grid)
a fast，smallest cost Astar search For moba，mmo game even for Autonomous driving and so on...
# Use
![2](https://github.com/user-attachments/assets/32799ddc-4de8-4835-8c7c-0e220ff4218c)

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
# Test:1000*1000 map(unity map)

green cube is obstalce,I scan this map by unity ray and then bulid a obstacle index json file to replace all obstacle cube:

![6](https://github.com/user-attachments/assets/9b366fee-f4ad-4bf0-9394-fcf99f0f3524)

obstacle json map is in MapData file,like this:

![1](https://github.com/user-attachments/assets/78442000-6e67-4788-883c-6defd7d2061a)

main:

```go
package main

import (
	"Go_Server/DataMgr"
	"fmt"
	jsoniter "github.com/json-iterator/go"
	"io"
	"os"
)

var Json = jsoniter.ConfigCompatibleWithStandardLibrary // 使用兼容标准库的配置
// ObstacleIndex json

// set Obstacle from json
var manager = DataMgr.NewMapManager(1000, 1000)
type ObstacleIndex struct {
	I int `json:"i"`
	J int `json:"j"`
}

func loadObstacleJson() {
	//read json obstacle
	file, err := os.Open("MapData/mapObstacle2.json")
	if err != nil {
		fmt.Println("Error opening file:", err)
		return
	}
	defer file.Close()
	// real Obstacle json file
	byteValue, _ := io.ReadAll(file)
	// load Obstacle json by json-iterator
	err = Json.Unmarshal(byteValue, &ObstacleIndexSlice)
	if err != nil {
		fmt.Println("Error parsing JSON:", err)
		return
	}
	// SetObstacle
	for _, obstacle := range ObstacleIndexSlice {
		manager.SetObstacle(obstacle.I, obstacle.J)
	}
}

// ObstacleIndexSlice save json ObstacleIndex
var ObstacleIndexSlice = make([]ObstacleIndex, 0, 100000)

func main() {
	//---------------------------------------load json Obstacle
	loadObstacleJson()
	//SetObstacle
	if manager.PathFind(0, 0, 930, 910, true, true, true) {
		for _, val := range manager.SmoothValType.SmoothFinalIndex {
			fmt.Println("x:", manager.FinalPathList[val].X, "\ty:", manager.FinalPathList[val].Y)
		}
	}
}
```
result map:


# My Astar Rules

8 dir:

![3](https://github.com/user-attachments/assets/3553d0af-796c-441a-9808-95a5875c0a58)

# obstacle judge Rules(white circle is obstacle,if some one is obstacle,failed to pass)

1.Normal

![4](https://github.com/user-attachments/assets/9bbe4d9b-1cb9-4b15-9b96-551dfa9595e3)

2.y=kx+b(check x line and y line,if a point around 4 obstacles,judge this 4 obstacles)

like there 4 triangles(different shape means obstacle)

![5](https://github.com/user-attachments/assets/c4d8eaf7-5e0a-4947-a1e3-59353840eded)



