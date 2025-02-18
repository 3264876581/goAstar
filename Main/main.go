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
	file, err := os.Open("MapData/mapObstacle_Test.json")
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
	if manager.PathFind(437, 428, 0, 0, true, true, true) {
		for _, val := range manager.SmoothValType.SmoothFinalIndex {
			fmt.Println("x:", manager.FinalPathList[val].X, "\ty:", manager.FinalPathList[val].Y)
		}
	}

	//reader := bufio.NewReader(os.Stdin)
	//for {
	//	fmt.Println("Press Enter to find path...")
	//	_, err := reader.ReadString('\n')
	//	if err != nil {
	//		fmt.Println("Error reading input:", err)
	//		continue
	//	}
	//}
	////
	//manager.SetObstacle(2, 2)
	//manager.SetObstacle(3, 2)
	//manager.SetObstacle(4, 2)
	//manager.SetObstacle(2, 3)
	//manager.SetObstacle(3, 3)
	//manager.SetObstacle(4, 3)
	//manager.SetObstacle(2, 4)
	//manager.SetObstacle(3, 4)
	//manager.SetObstacle(4, 4)
	//
	//manager.SetObstacle(2, 6)
	//manager.SetObstacle(3, 6)
	//manager.SetObstacle(4, 6)
	//manager.SetObstacle(2, 7)
	//manager.SetObstacle(3, 7)
	//manager.SetObstacle(4, 7)
	//manager.SetObstacle(2, 8)
	//manager.SetObstacle(3, 8)
	//manager.SetObstacle(4, 8)
	//
	//manager.PathFind(0, 0, 999, 999, true, true)
	//---------------------------------------read json map
	//read json obstacle
	//file, err := os.Open("MapData/mapObstacle2.json")
	//if err != nil {
	//	fmt.Println("Error opening file:", err)
	//	return
	//}
	//defer file.Close()
	//
	//// real file
	//byteValue, _ := io.ReadAll(file)
	//
	//// load json by json-iterator
	//
	//err = Json.Unmarshal(byteValue, &ObstacleIndexSlice)
	//if err != nil {
	//	fmt.Println("Error parsing JSON:", err)
	//	return
	//}
	//
	//// SetObstacle
	//for _, obstacle := range ObstacleIndexSlice {
	//	manager.SetObstacle(obstacle.I, obstacle.J)
	//}

	//for {
	//	fmt.Println("Press Enter to find path...")
	//	_, err := reader.ReadString('\n')
	//	if err != nil {
	//		fmt.Println("Error reading input:", err)
	//		continue
	//	}
	//
	//	// 提示用户输入 x1, y1, x2, y2
	//	fmt.Print("Enter x1: ")
	//	x1, err := readInt(reader)
	//	if err != nil {
	//		fmt.Println("Error reading x1:", err)
	//		continue
	//	}
	//
	//	fmt.Print("Enter y1: ")
	//	y1, err := readInt(reader)
	//	if err != nil {
	//		fmt.Println("Error reading y1:", err)
	//		continue
	//	}
	//
	//	fmt.Print("Enter x2: ")
	//	x2, err := readInt(reader)
	//	if err != nil {
	//		fmt.Println("Error reading x2:", err)
	//		continue
	//	}
	//
	//	fmt.Print("Enter y2: ")
	//	y2, err := readInt(reader)
	//	if err != nil {
	//		fmt.Println("Error reading y2:", err)
	//		continue
	//	}
	//
	//	// 调用 PathFind 函数
	//	manager.PathFind(x1, y1, x2, y2, true, true)
	//}
	//slice1 := []int{1, 2, 3, 4, 5}
	//slice2 := []int{5, 4, 3}
	//copy(slice2, slice1)
	//修改slice2 并不会对slice1造成影响
	//slice2[0] = 10
	//fmt.Println(slice1, slice2)
	//[1 2 3 4 5] [10 2 3]
	//s1 := []int{1, 2, 3}
	//s2 := make([]int, 3)
	//copy(s2, s1)
	//fmt.Println(&s1[0], &s2[0])
	//0xc0000140d8 0xc000014228
}

//func readInt(reader *bufio.Reader) (int, error) {
//	text, err := reader.ReadString('\n')
//	if err != nil {
//		return 0, err
//	}
//	text = strings.TrimSpace(text)
//	return strconv.Atoi(text)
//}
