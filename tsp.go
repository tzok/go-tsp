/* vim: set filetype=go : */
package main

import (
    "fmt"
    "io/ioutil"
    "os"
    "rand"
    "sort"
    "strconv"
    "strings"
    "time"
)

func loadInstance(path string) (matrix [][]int) {
    b, err := ioutil.ReadFile(path)
    if err != nil {
        fmt.Println(err)
        return
    }

    var (
        started bool
        row, column int
    )
    for _, line := range strings.Split(string(b), "\n", -1) {
        fields := strings.Fields(line)
        if len(fields) > 0 {
            if !started {
                if fields[0] == "DIMENSION:" {
                    n, err := strconv.Atoi(fields[1])
                    if err != nil {
                        fmt.Println(err)
                    }
                    matrix = make([][]int, n)
                    for i := 0; i < n; i++ {
                        matrix[i] = make([]int, n)
                    }
                }
                if fields[0] == "EDGE_WEIGHT_SECTION" {
                    started = true
                }
            } else {
                if fields[0] == "EOF" {
                    break
                }
                for _, v := range fields {
                    matrix[column][row], err = strconv.Atoi(v)
                    if err != nil {
                        fmt.Println(err)
                        return
                    }
                    row++
                    if row == len(matrix) {
                        row = 0
                        column++
                    }
                }
            }
        }
    }
    return
}

func random(matrix [][]int) (tour []int, best int) {
    const maxIters = 1e6

    best = 1e6
    for i := 0; i < maxIters; i++ {
        perm := rand.Perm(len(matrix))
        length := matrix[perm[len(matrix)-1]][perm[0]]
        for j := 1; j < len(matrix); j++ {
            length += matrix[perm[j-1]][perm[j]]
        }
        if length < best {
            tour, best = perm, length
        }
    }
    return
}

func heuristic(matrix [][]int) (tour []int, best int) {
    best = 1e6

    free := make(map[int]bool)
    for i := 0; i < len(matrix); i++ {
        for j := 0; j < len(matrix); j++ {
            free[j] = j != i
        }

        curr := make([]int, len(matrix))
        curr[0] = i

        index := 0
        total := 0
        for index + 1 < len(matrix) {
            var min  int = 1e6
            var argmin int
            for j := 0; j < len(matrix); j++ {
                if free[j] {
                    dist := matrix[curr[index]][j]
                    if dist < min {
                        min = dist
                        argmin = j
                    }
                }
            }
            index++
            curr[index] = argmin
            free[argmin] = false
            total += min
        }

        total += matrix[curr[len(matrix)-1]][curr[0]]
        if total < best {
            tour, best = curr, total
        }
    }
    return
}


type arc struct {
    u, v int
}

type neigh struct {
    i, j, delta int
}

type neighbourhood []neigh

func (n neighbourhood) Len() int {
    return len(n)
}

func (n neighbourhood) Less(i, j int) bool {
    return n[i].delta < n[j].delta
}

func (n neighbourhood) Swap(i, j int) {
    n[i], n[j] = n[j], n[i]
}

func n2opt(matrix [][]int, arcs []arc, greedy bool) (neighs neighbourhood) {
    n := len(matrix)
    neighs = make(neighbourhood, 0)
    for i := 0; i < n; i++ {
        for j := 0; j < n; j++ {
            if i == j {
                continue
            }
            delta := 0
            delta -= matrix[arcs[i].u][arcs[i].v]
            delta -= matrix[arcs[j].u][arcs[j].v]
            delta += matrix[arcs[i].u][arcs[j].u]
            delta += matrix[arcs[i].v][arcs[j].v]
            for k := (i+1)%n; k != j; k = (k+1)%n {
                delta -= matrix[arcs[k].u][arcs[k].v]
                delta += matrix[arcs[k].v][arcs[k].u]
            }
            if greedy && delta < 0 {
                neighs = []neigh{neigh{i, j, delta}}
                return
            }
            neighs = append(neighs, neigh{i, j, delta})
        }
    }
    sort.Sort(neighs)
    return
}

func path2arcs(path []int) (arcs []arc) {
    n := len(path)
    arcs = make([]arc, n)
    for i := 0; i < n; i++ {
        arcs[i] = arc{path[i], path[(i+1)%n]}
    }
    return
}

func applyMove(in []arc, move neigh) (out []arc) {
    out = make([]arc, len(in))
    i, j, k, n := move.i, move.j, 0, len(in)
    out[k] = arc{in[i].u, in[j].u}
    k++
    for l := (j-1+n)%n; l != i; l = (l-1+n)%n {
        out[k] = arc{in[l].v, in[l].u}
        k++
    }
    out[k] = arc{in[i].v, in[j].v}
    k++
    for l := (j+1)%n; l != i; l = (l+1)%n {
        out[k] = in[l]
        k++
    }
    return
}

func arcs2path(arcs []arc) (path []int) {
    path = make([]int, len(arcs))
    for i, arc := range arcs {
        path[i] = arc.u
    }
    return
}

func localSearch(matrix [][]int, greedy bool) (path []int, best int) {
    path, best = heuristic(matrix[:])
    arcs := path2arcs(path[:])
    for {
        n := n2opt(matrix[:], arcs[:], greedy)
        if n[0].delta >= 0 {
            break
        }
        arcs = applyMove(arcs[:], n[0])
        best += n[0].delta
    }
    path = arcs2path(arcs[:])
    return
}

func steepest(matrix [][]int) (path []int, best int) {
    return localSearch(matrix[:], false)
}

func greedy(matrix [][]int) (path []int, best int) {
    return localSearch(matrix[:], true)
}

var maxIters int
var tabuLength int
var tabuList []int

func isTabu(arcs []arc, move neigh, iter int) (result bool) {
    result = iter - tabuList[arcs[move.i].u] < tabuLength
    result = result || iter - tabuList[arcs[move.i].v] < tabuLength
    result = result || iter - tabuList[arcs[move.j].u] < tabuLength
    result = result || iter - tabuList[arcs[move.j].v] < tabuLength
    return
}

func makeTabu(arcs []arc, move neigh, iter int) {
    tabuList[arcs[move.i].u] = iter
    tabuList[arcs[move.i].v] = iter
    tabuList[arcs[move.j].u] = iter
    tabuList[arcs[move.j].v] = iter
}

func tabuSearch(matrix [][]int) (path []int, best int) {
    path = make([]int, len(matrix))
    best = 0
    for i := 0; i < len(matrix); i++ {
        path[i] = i
        best += matrix[i][(i+1)%len(matrix)]
    }

    path, best = heuristic(matrix[:])
    arcs := path2arcs(path[:])
    maxIters = 1 << 8
    tabuList = make([]int, len(matrix))
    tabuLength = len(matrix)
    iter := 0
    curr := best
    for i := 0; i < maxIters; i++ {
        n := n2opt(matrix[:], arcs[:], false)
        for _, v := range n {
            if !isTabu(arcs[:], v, iter) || curr + v.delta < best {
                makeTabu(arcs[:], v, iter)
                arcs = applyMove(arcs[:], v)

                curr += v.delta
                if v.delta < 0 {
                    if curr < best {
                        best = curr
                        path = arcs2path(arcs[:])
                    }
                    // i = 0
                }
                break
            }
        }
        iter++
    }
    return
}

func main() {
    if len(os.Args) != 3 {
        fmt.Println("Usage: ./tsp <INSTANCE> <ALGORITHM>")
        return
    }

    algs := make(map[string] func([][]int) ([]int, int))
    algs["RANDOM"] = random
    algs["HEURISTIC"] = heuristic
    algs["STEEPEST"] = steepest
    algs["GREEDY"] = greedy
    algs["TABU"] = tabuSearch

    if _, ok := algs[os.Args[2]]; !ok {
        fmt.Println("Unknown algorithm:", os.Args[2])
        return
    }

    rand.Seed(time.Seconds())
    matrix := loadInstance(os.Args[1])
    tour, best := algs[os.Args[2]](matrix[:])
    fmt.Println(best, tour)
}
