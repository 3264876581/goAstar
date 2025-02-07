# Astar(Grid)
A fast(1k * 1k cost avg 1-2ms,10k * 10k cost avg 15ms)，smallest cost Astar search For moba，mmo game even for Autonomous driving and so on...
# Easy Use
![11](https://github.com/user-attachments/assets/d5bb2627-075f-417e-9178-79a3a9bda85c)

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

# Result
manager.FinalPathList-----save Inflection node 

manager.SmoothValType.SmoothFinalIndex -----save smallest cost index of manager.FinalPathList

![1](https://github.com/user-attachments/assets/5681ae5d-20b0-444d-b9b6-e7f3a95152ea)

you can judge PathFind flag(if success find) then loop SmoothFinalIndex like this to use smallest cost index in FinalPathList:

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

![1](https://github.com/user-attachments/assets/9d072ccf-3409-478f-8e2a-c72b90130fc2)


obstacle json map is in MapData file,like this:

![1](https://github.com/user-attachments/assets/78442000-6e67-4788-883c-6defd7d2061a)

main code:

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
result map(unity):

![2](https://github.com/user-attachments/assets/030cf164-c7d5-45b4-aacd-c29cb5320f2f)

result map(go print):

![dcd10e77-a5cb-4663-a17c-ca963993ad63](https://github.com/user-attachments/assets/bf67c48e-189c-41f3-88e4-0ae98b1ae941)
![cb8bef9b-fbf1-4102-b5d1-531a727bc6a4](https://github.com/user-attachments/assets/89c289f6-2da4-4d97-b5a6-566e36c34313)
![ca4fd4af-6215-498c-870e-1136399b2342](https://github.com/user-attachments/assets/ca21655f-bea6-4a4a-993e-922ff1fccc1f)
![5ab63e1d-5ecd-4a31-a1e9-8e8e71d6dd4b](https://github.com/user-attachments/assets/0a6f8931-2be0-45be-80fc-b9f0e6a6b0b7)
![9726f768-53ef-412e-ada8-36fc73e5d209](https://github.com/user-attachments/assets/e47c2786-8c60-40c0-94c0-66c73dd263f3)
![315a72e1-578a-404d-bad2-6f8699b80106](https://github.com/user-attachments/assets/091a2cf9-d098-4776-ae4a-ef81b63c0529)
![9e91af1f-3e20-4b8b-882d-562bcd1c62d6](https://github.com/user-attachments/assets/e0425280-0dae-44e7-a4af-c12ba4e89d30)


print(smallest cost index)： 0 - 2 - 9 - 10 - 11

![3](https://github.com/user-attachments/assets/4d0f7380-9429-4f49-aa34-81827c81b43b)


# My Astar Rules

8 dir:

![3](https://github.com/user-attachments/assets/3553d0af-796c-441a-9808-95a5875c0a58)

# obstacle judge Rules
# if has one circle is obstacle , pass failed

1.Normal

![4](https://github.com/user-attachments/assets/9bbe4d9b-1cb9-4b15-9b96-551dfa9595e3)

2.y=kx+b(check x line and y line)

if a point around 4 obstacles，judge this 4 obstacles around it

else judge 2 obstacles(ud and down or left and right)

around 4 obstacles：like there 4 triangles(different shape means obstacle)

around 2 obstacles: like 2 cicle at last img

![5](https://github.com/user-attachments/assets/c4d8eaf7-5e0a-4947-a1e3-59353840eded)



