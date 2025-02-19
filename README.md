# Astar(Grid)
A fast(1k * 1k cost avg 1-2ms,10k * 10k cost avg 15ms)，smallest cost Astar search For moba，mmo game even for Autonomous driving and so on...
# Easy Use
build a map manager
```go
var manager = DataMgr.NewMapManager(1000, 1000)
```
![11](https://github.com/user-attachments/assets/d5bb2627-075f-417e-9178-79a3a9bda85c)

findPath 
```go
manager.PathFind(0, 0, 9, 9, true, true, true)
```
![2](https://github.com/user-attachments/assets/32799ddc-4de8-4835-8c7c-0e220ff4218c)

set Obstacle parms(x y) 
```go
manager.SetObstacle(0, 0)
```

set Road parms(x y)
```go
manager.SetRoad(0, 0)
```

# Example main

if you want to change Obstacle or road，please do it before PathFind

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

![11](https://github.com/user-attachments/assets/a36c102b-2581-49f8-88fd-0dda398b52fe)

manager.FinalPathList -> save Inflection node (there is 0 1 2 3)

manager.SmoothValType.SmoothFinalIndex (there is 0 1 3) -> save smallest cost index of manager.FinalPathList

![13](https://github.com/user-attachments/assets/e5c17b3c-e038-4279-bfdd-edcb6a29244d)


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
	if manager.PathFind(0, 0, 9, 9, true, true, true) {
		fmt.Println("--loop manager.SmoothValType.SmoothFinalIndex and print manager.FinalPathList Node X,Y--")
		for _, val := range manager.SmoothValType.SmoothFinalIndex {
			fmt.Println("x:", manager.FinalPathList[val].X, "\ty:", manager.FinalPathList[val].Y)
		}
	}
 }
```
![15](https://github.com/user-attachments/assets/aca0d0bc-39aa-4b41-b86d-c6310fb32c97)

# Test:1000*1000 map(unity map)

scan this map by unity ray and then create a obstacle index json file to replace all obstacle :

![134](https://github.com/user-attachments/assets/35d0d13b-21d8-49df-ad58-ba9d9cab03cd)


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
	if manager.PathFind(0, 0, 999, 999, true, true, true) {
		fmt.Println("--loop manager.SmoothValType.SmoothFinalIndex and print manager.FinalPathList Node X,Y--")
		for _, val := range manager.SmoothValType.SmoothFinalIndex {
			fmt.Println("x:", manager.FinalPathList[val].X, "\ty:", manager.FinalPathList[val].Y)
		}
	}
}
```
result map(unity):

![11](https://github.com/user-attachments/assets/cf7cc732-351d-4e1b-b240-ea1ce036269b)

result map(go print):

![23309807-424b-4afd-b752-21701caf33f3](https://github.com/user-attachments/assets/4c9c0560-ba98-42c2-ab2c-a552eb0af7e3)
![1816b973-d044-4d26-a59b-52e56b59474d](https://github.com/user-attachments/assets/1a00c33a-ac81-4876-b1b0-0c85ebc8c241)
![8cf46501-7e7a-43fd-aab1-c027bc9fd308](https://github.com/user-attachments/assets/8e187ef2-0bd5-4fcd-afc1-5f65f2b1a31d)
![461feb5a-8420-4a93-913d-4d9bcf4c1df1](https://github.com/user-attachments/assets/633a0ff2-34a7-401e-82b8-3aa539b5d17e)
![6fffb1cd-79b1-4c57-a50f-f68c93abcb2b](https://github.com/user-attachments/assets/d187e370-5667-4e8b-8dcc-b6abed9287ff)


print(smallest cost index)： 0 - 1 - 2 - 3 - 5

![999](https://github.com/user-attachments/assets/6867feb7-a14e-4a42-a2d1-e83c24d824e6)


# My Astar Rules

8 dir:

![3](https://github.com/user-attachments/assets/3553d0af-796c-441a-9808-95a5875c0a58)

# About Smooth path：obstacle judge Rules
# if has one circle is obstacle , pass failed

1.Normal

![4](https://github.com/user-attachments/assets/9bbe4d9b-1cb9-4b15-9b96-551dfa9595e3)

2.y=kx+b(check x line and y line)

if a point around 4 obstacles，judge this 4 obstacles around it

else judge 2 obstacles(ud and down or left and right)

around 4 obstacles：like there 4 triangles(different shape means obstacle)

around 2 obstacles: like 2 cicle Below in this photo

![5](https://github.com/user-attachments/assets/c4d8eaf7-5e0a-4947-a1e3-59353840eded)



