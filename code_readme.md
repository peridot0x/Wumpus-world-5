**************
## planner.py A* algorithm code [TODO 4]
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

************************************************************

  
##Agent.py update_knowledge (TODO 6)
## 알고리즘 설명: 지식 갱신 (`agent.update_knowledge`, TODO 6)

### 역할
KB에 쌓인 논리식에 대해 **"이 칸에 구덩이가 있는가, 웜퍼스가 있는가"를 ASK로 묻고, 증명된 결과만 집합에 저장**한다.
행동 결정(`choose_plan`)은 KB를 직접 읽지 않고 이 집합들만 사용한다.

| 집합 | 의미 |
|---|---|
| `not_pit` | 구덩이가 없다고 **증명된** 칸 |
| `known_pit` | 구덩이가 있다고 **증명된** 칸 |
| `not_wumpus` | 웜퍼스가 없다고 증명된 칸 |
| `known_wumpus` | 웜퍼스 위치 (모르면 `None`) |

`safe` 속성은 `not_pit`과 `not_wumpus`(웜퍼스가 죽었다면 `not_pit`만)에서 자동으로 계산된다.

### 구현 코드
```python
def update_knowledge(self):
    if not self.dirty:
        return
    self.dirty = False
    for c in self.cells:
        # ---- 구덩이
        if c not in self.not_pit and c not in self.known_pit:
            if self.kb.ask(Not(P(c))):
                self.not_pit.add(c)
            elif self.kb.ask(P(c)):
                self.known_pit.add(c)
        # ---- 웜퍼스 (살아 있고, 아직 위치를 모를 때만)
        if not self.wumpus_dead and self.known_wumpus is None and c not in self.not_wumpus:
            if self.kb.ask(Not(W(c))):
                self.not_wumpus.add(c)
            elif self.kb.ask(W(c)):
                self.known_wumpus = c
                for o in self.cells:            # 웜퍼스는 하나뿐 → 나머지 칸은 모두 ¬W
                    if o != c:
                        self.kb.tell(Not(W(o)))
                        self.not_wumpus.add(o)
```

### 핵심 아이디어: 세 가지 결과를 구분한다
`ask`가 거짓이라는 것은 "있다"는 뜻이 아니라 **"없다고 증명할 수 없다"** 는 뜻이다. 그래서 질문을 두 번 한다.

| `ask(¬P)` | `ask(P)` | 의미 | 처리 |
|---|---|---|---|
| 참 | (묻지 않음) | 구덩이 없음이 증명됨 | `not_pit`에 추가 |
| 거짓 | 참 | 구덩이 있음이 증명됨 | `known_pit`에 추가 |
| 거짓 | 거짓 | **아직 모름** | 아무것도 하지 않음 |

웜퍼스도 같은 3분기 구조이다. "모르는 칸"을 안전으로도, 위험으로도 확정하지 않는 것이 에이전트의 **건전성**(안전 정책 사망 0%)을 지킨다.

### 줄별 설명
| 코드 | 기능 |
|---|---|
| `if not self.dirty: return` | KB가 바뀌지 않았으면 다시 계산하지 않는다 (ASK 횟수 절약) |
| `self.dirty = False` | 이번 갱신으로 최신 상태가 반영됨을 표시 |
| `for c in self.cells` | 모든 칸을 차례로 검사 |
| `c not in not_pit and c not in known_pit` | 이미 결론이 난 칸은 건너뜀. 증명된 사실은 KB가 커져도 뒤집히지 않으므로(단조성) 다시 물을 필요가 없다 |
| `ask(Not(P(c)))` 참 → `not_pit` | 구덩이 없음이 증명됨 |
| `elif ask(P(c))` 참 → `known_pit` | 구덩이 있음이 증명됨 (앞 질문이 거짓일 때만 호출) |
| `not wumpus_dead and known_wumpus is None and c not in not_wumpus` | 웜퍼스가 살아 있고, 위치를 모르며, 이 칸이 아직 미확정일 때만 질문 |
| `ask(Not(W(c)))` 참 → `not_wumpus` | 이 칸에 웜퍼스 없음 증명 |
| `elif ask(W(c)))` 참 → `known_wumpus = c` | 웜퍼스 위치 확정 |
| `for o ... tell(Not(W(o)))`, `not_wumpus.add(o)` | "웜퍼스는 하나뿐"이라는 세계 규칙 적용. 확정된 칸 `c`를 제외한 모든 칸을 `¬W`로 TELL하고 `not_wumpus`에 추가 |

"웜퍼스는 하나뿐"을 KB 절로 넣지 않고 위치 확정 시점에 코드로 처리하는 이유는, 모든 `W` 기호를 묶는 절이 들어가면 `relevant_clauses`가 거의 전체 KB를 끌어와 ASK가 느려지기 때문이다.

### 동작 예: 교재 그림 7.4 상황
(2,2)에서 냄새가 없고 (1,2)에서만 냄새가 나는 경우이다.
1. (2,2)에 냄새가 없으므로 (1,2), (2,1), (3,2), (2,3), (2,2)에는 웜퍼스가 없다.
2. (1,2)의 냄새 규칙은 `W12 ∨ W11 ∨ W22 ∨ W13`이다. 앞의 세 칸이 모두 거짓이므로 `W13`만 남는다.
3. `ask(¬W13)`은 거짓, `ask(W13)`은 참이므로 `known_wumpus = (1,3)`이 된다.
4. 나머지 15칸이 모두 `not_wumpus`가 되고, 그중 `not_pit`인 칸이 안전 칸이 된다.

### 설계상 주의점
- **반복 중 `known_wumpus`가 바뀌어도 안전하다.** 이후 칸은 `known_wumpus is None`이 거짓이라 건너뛰고, 이미 지나간 칸은 ASK 결과로 `not_wumpus`에 들어 있거나 확정 시점에 한꺼번에 채워진다.
- **확정된 칸 자신은 `not_wumpus`에 넣지 않는다.** (`o != c`) 그래서 그 칸은 `safe`에서 빠져 에이전트가 들어가지 않는다.
- **`dirty`와의 관계**: 함수 안의 `tell(¬W)`는 `dirty`를 다시 켜지 않는다. 이미 결론이 반영된 TELL이어서 다시 계산할 필요가 없다.
- **효율**: 칸마다 ASK는 최대 4번(구덩이 2, 웜퍼스 2)이지만 확정된 칸은 건너뛰고, `relevant_clauses`가 관련 절만 추려서 ASK 1회가 약 0.01 ms이다.

### 흔한 실수
| 실수 | 결과 |
|---|---|
| `ask(¬P)`가 거짓일 때 `else`로 `known_pit`에 넣음 | 모르는 칸을 위험으로 오판해 탐험 기회를 놓침 |
| `ask(P)`가 거짓이면 안전으로 처리 | 증명되지 않은 칸에 들어가 사망 |
| 위치 확정 후 `not_wumpus`에 `c`까지 포함 | 웜퍼스 칸을 안전으로 판단해 사망 |
| 이미 확정된 칸도 매번 ASK | 결과는 같지만 실행이 느려짐 |

### 검증 결과
세계 3600개(DPLL 3000개, TT 600개, seed 1000 이상)에서 에이전트가 확정한 집합을 실제 세계와 대조했다.

| 검사 | 틀린 건수 |
|---|---|
| `not_pit`인데 실제 구덩이 | 0 |
| `known_pit`인데 실제 구덩이 아님 | 0 |
| `not_wumpus`인데 실제 웜퍼스 칸 | 0 |
| `known_wumpus`가 실제 위치와 다름 | 0 (확정한 1085개 에피소드 모두 정확) |
| 안전 정책 사망 | 0 |

관련 절 추출(가지치기)을 켜고 끈 두 경우의 지식 집합은 400개 세계에서 모두 같았다.

### 한계
KB에 "웜퍼스가 최소 한 마리 있다"는 절이 없어서, 3000개 중 11개 에피소드에서는 나머지 15칸이 모두 `¬W`인데도 남은 칸을 웜퍼스로 확정하지 못했다.
그 칸은 `not_wumpus`에 없으므로 `safe`에서 빠져 **안전에는 영향이 없고**, 사격할 때 위치가 확정된 경우의 분기를 타지 못하는 효율상의 손실만 있다.
반복문 뒤에 "`not_wumpus`에 없는 칸이 하나뿐이면 그 칸을 `known_wumpus`로 정한다"를 추가하면 보완할 수 있다(개선 방향).

### 테스트
```
python test_agent.py     # TODO 5~7 에이전트 (그림 7.2), (무작위 200개, 건전성)  → PASS
```
건전성 테스트는 KB의 모든 절이 실제 세계에서 참인지, 안전 정책에서 사망하지 않는지를 검사한다.
