package DataMgr

type Heap struct {
	smallestCostIndex int
	leftIndex         int
	rightIndex        int
	//swap temporary
	temporaryVal *Node
}

// HeapIfy heapIfy a smallest heap for openList
//func (mapManager *MapManager) HeapIfy(beginIndex int) {
//	mapManager.heap.smallestCostIndex = beginIndex
//	mapManager.heap.leftIndex = 2*beginIndex + 1
//	mapManager.heap.rightIndex = 2*beginIndex + 2
//	//judge left Index and smallest Index
//	if mapManager.heap.leftIndex < len(mapManager.openList)-1 && mapManager.openList[mapManager.heap.leftIndex].f < mapManager.openList[mapManager.heap.smallestCostIndex].f {
//		mapManager.heap.smallestCostIndex = mapManager.heap.leftIndex
//	}
//	//judge smallest Index and right Index
//	if mapManager.heap.rightIndex < len(mapManager.openList)-1 && mapManager.openList[mapManager.heap.rightIndex].f < mapManager.openList[mapManager.heap.smallestCostIndex].f {
//		mapManager.heap.smallestCostIndex = mapManager.heap.rightIndex
//	}
//	//if smallestIndex change
//	if mapManager.heap.smallestCostIndex != beginIndex {
//		mapManager.swapOpenListElement(mapManager.heap.smallestCostIndex, beginIndex)
//		mapManager.HeapIfy(mapManager.heap.smallestCostIndex)
//	}
//}

//func (mapManager *MapManager) swapOpenListElement(aIndex, bIndex int) {
//	mapManager.heap.temporaryVal = mapManager.openList[aIndex]
//	mapManager.openList[aIndex] = mapManager.openList[bIndex]
//	mapManager.openList[bIndex] = mapManager.heap.temporaryVal
//}
//
//func (mapManager *MapManager) BuildSmallestCostHeap() {
//	for i := len(mapManager.openList)/2 - 1; i >= 0; i-- {
//		mapManager.HeapIfy(i)
//	}
//}
//
//func (mapManager *MapManager) SortCostHeapOfEndIndexIsSmallest() {
//	for i := len(mapManager.openList)/2 - 1; i >= 0; i-- {
//		mapManager.HeapIfy(i)
//	}
//	//
//	for i := len(mapManager.openList) - 1; i >= 0; i-- {
//		mapManager.swapOpenListElement(0, i)
//		mapManager.HeapIfy(i)
//	}
//}
