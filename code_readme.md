**************
## planner.py A* algorithm code
### 코드 동작 (줄별 설명)

```python
tie = count()
```
0, 1, 2, …를 차례로 내주는 카운터를 만든다. 힙 항목의 f가 같을 때 삽입 순서로 순서를 정하게 해서, state끼리 비교하는 일을 막고 실행 결과를 결정적으로 만든다.

```python
g0 = {start: 0}
```
상태별로 **지금까지 찾은 최소 g**를 기록한다. 시작 상태의 g는 0이다.

```python
parent = {start: None}
```
각 상태가 어느 상태에서 어떤 동작으로 왔는지 기록한다. 시작 상태는 `None`이며, 경로를 복원할 때 이것이 종료 조건이 된다.

```python
open_heap = [(h(start), next(tie), 0, start)]
```
시작 상태를 힙에 넣는다. 항목은 `(f, 카운터, g, state)`이고, 시작 상태는 g = 0이므로 f = h(start)이다.

```python
while open_heap:
    f, _, g, state = heapq.heappop(open_heap)
```
힙이 빌 때까지 반복한다. f가 가장 작은 항목을 꺼낸다. `f`는 정렬에만 쓰이고 이후에는 쓰지 않으며, `_`는 카운터이다.

```python
    if g > g0.get(state, float("inf")):
        continue
```
**낡은 항목 제거(lazy deletion).** `heapq`는 이미 들어간 항목을 수정할 수 없다. 같은 상태를 더 짧은 경로로 다시 발견하면 새 항목을 추가하고 예전 항목은 남겨 둔다. 예전 항목이 나오면 g가 기록된 최소값보다 크므로 여기서 버린다.

```python
    if stats is not None:
        stats["expanded"] = stats.get("expanded", 0) + 1
```
확장한 노드 수를 센다. 탐색 효율을 분석할 때 쓴다.

```python
    if goal_test(state):
        plan = []
        while parent[state] is not None:
            state, act = parent[state][0], parent[state][1]
            plan.append(act)
        return plan[::-1]
```
목표 상태이면 `parent`를 따라 시작 상태까지 거슬러 올라가며 동작을 모으고, 거꾸로 모았으므로 `[::-1]`로 뒤집어 반환한다. 목표 검사는 노드를 **꺼낼 때** 해야 최적성이 보장된다. 시작 상태가 이미 목표이면 반복문을 건너뛰어 빈 리스트 `[]`를 반환한다.

```python
    for act, nxt in successors(state, allowed):
```
현재 상태에서 할 수 있는 동작과 그 결과 상태를 하나씩 가져온다. `TurnLeft`, `TurnRight`는 항상 가능하고, `Forward`는 다음 칸이 `allowed`에 있을 때만 가능하다. 이 때문에 안전하다고 증명된 칸으로만 경로가 만들어진다.

```python
        ng = g + 1
```
이웃까지의 새 비용이다. 모든 동작의 비용이 1이다.

```python
        if ng < g0.get(nxt, float("inf")):
```
처음 보는 상태이거나(기록이 없으면 무한대), 이전에 찾은 것보다 짧은 경로일 때만 갱신한다.

```python
            g0[nxt] = ng
            parent[nxt] = (state, act)
            heapq.heappush(open_heap, (ng + h(nxt), next(tie), ng, nxt))
```
최소 g를 갱신하고, 어디서 어떤 동작으로 왔는지 기록하고, `f = ng + h(nxt)`로 힙에 넣는다.

```python
return None
```
힙이 비도록 목표에 도달하지 못했다는 뜻이다. 안전한 칸만으로는 갈 방법이 없을 때 해당한다.

### 동작 예시: (1,1)에서 동쪽을 보고 (2,2)로

`allowed = {(1,1), (2,1), (3,1), (1,2), (2,2)}`이고, 상태는 `(x, y, 방향)`이다. 시작 상태 (1,1,0)의 h는 |1−2| + |1−2| = 2이다.

| 순서 | 꺼낸 상태 | g | f | 새로 넣은 상태 (동작, g, f) |
|---|---|---|---|---|
| 1 | (1,1,0) | 0 | 2 | (1,1,1) TurnLeft 1/3, (1,1,3) TurnRight 1/3, (2,1,0) Forward 1/2 |
| 2 | (2,1,0) | 1 | 2 | (2,1,1) TurnLeft 2/3, (2,1,3) TurnRight 2/3, (3,1,0) Forward 2/4 |
| 3 | (1,1,1) | 1 | 3 | (1,1,2) TurnLeft 2/4, (1,2,1) Forward 2/3 |
| 4 | (1,1,3) | 1 | 3 | 없음 (새 상태가 모두 더 짧지 않음) |
| 5 | (2,1,1) | 2 | 3 | (2,1,2) TurnLeft 3/4, **(2,2,1) Forward 3/3** |
| 6 | (2,1,3) | 2 | 3 | 없음 |
| 7 | (1,2,1) | 2 | 3 | (1,2,2) TurnLeft 3/4, (1,2,0) TurnRight 3/4 |
| 8 | (2,2,1) | 3 | 3 | 목표 → 경로 복원 |

복원 결과는 `['Forward', 'TurnLeft', 'Forward']`로 동작 3개이며, 이것이 최소이다. f가 같은 항목끼리는 먼저 들어간 것이 먼저 나온다(카운터의 효과).

### 이 코드에서 확인할 점
- `test_agent.py`의 TODO 4가 직선 경로, 최소 동작 수 3, 경로 없음(`None`)을 검사한다.
- 막힌 경우에는 `plan_route((1,1), 0, {(4,4)}, 일부 칸을 뺀 allowed)`처럼 호출해 `None`이 반환되는지 확인할 수 있다.
