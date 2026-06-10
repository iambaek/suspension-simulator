# 자동차 서스펜션 시뮬레이터 — 보고서 작성 컨텍스트

> 이 문서는 다른 Claude(또는 사람)에게 프로젝트를 빠르게 이해시켜 **고등학교 생기부 프로젝트 보고서** 작성을 돕기 위한 종합 컨텍스트입니다. 본문 마지막에 다른 Claude에게 바로 던질 수 있는 프롬프트 예시가 있습니다.

---

## 0. 핵심 링크

- **라이브 데모**: https://iambaek.github.io/suspension-simulator/
- **GitHub 레포(공개)**: https://github.com/iambaek/suspension-simulator
- **전체 소스 raw URL** (다른 Claude가 fetch 가능):
  https://raw.githubusercontent.com/iambaek/suspension-simulator/main/playground.html

전체 코드는 단일 HTML 파일(`playground.html`, 약 1800줄)이며 외부 라이브러리·빌드 없이 브라우저에서 바로 실행됩니다.

---

## 1. 프로젝트 개요

### 주제
자동차 서스펜션 시스템을 7자유도 풀카(full-car) 동역학 모델로 시뮬레이션하고, 일반적인 수동 서스펜션과 **분수미분(fractional-derivative) 능동 제어** 서스펜션의 승차감 차이를 정량적으로 비교하는 웹 시뮬레이터.

### 동기
- 일반 자동차는 코일 스프링과 댐퍼만으로 충격을 흡수하지만, 노면이 다양해질수록 한 가지 감쇠 계수로는 모든 충격을 잘 흡수하기 어렵다.
- **분수미분(Fractional Calculus)**은 "정수 차수가 아닌 임의의 실수 차수의 미분"을 다루는 수학 분야로, 신호의 과거 이력 전체를 가중치로 반영하는 **기억 효과(memory effect)**가 있다.
- 이 기억 효과를 제어력으로 활용하면 광대역 주파수에 동시에 대응할 수 있다는 가설을 시뮬레이션으로 검증해 보고 싶었다.

### 결과
- 동일한 장애물(좌우 다른 높이의 방지턱, 좌·우 한쪽 2연속 방지턱 등)을 같은 속도로 통과시키면서 모드별 차체 가속도 RMS를 비교 가능
- 분수미분 차수 α ∈ {0.5, 0.6, 0.7}을 실시간으로 바꿔가며 비교
- 장애물별로 평균/피크 RMS를 자동 측정해 표로 정리

---

## 2. 물리 모델 (7자유도 풀카)

### 상태 변수 (총 14개)
- **차체(sprung mass)**: 상하(heave) `z`, 피치 `θ`, 롤 `φ` — 각각 위치·속도 = 6개
- **바퀴(unsprung mass)** 4개: 각각 상하 위치·속도 = 8개

바퀴 인덱스 규약: `0=FL(앞왼), 1=FR(앞오), 2=RL(뒤왼), 3=RR(뒤오)`

### 차체 코너의 운동학 (소각 선형화)
각 바퀴 부착 지점의 차체 측 변위:
```
z_c[i]     = z + x_pos[i]·θ − y_pos[i]·φ
ż_c[i]     = ż + x_pos[i]·θ̇ − y_pos[i]·φ̇
```
부호 약속: `θ > 0`이면 앞이 들림(nose up), `φ > 0`이면 우측이 내려감.

### 운동 방정식 (각 코너 i에 대해)

스프링 힘, 댐퍼 힘, 능동 제어 힘:
```
F_s[i] = k_s · (z_c[i] − z_u[i])                  (스프링)
F_d[i] = c_s · (ż_c[i] − ż_u[i])                  (댐퍼)
F_a[i] = (제어 모드에 따라 결정)                  (액추에이터)
```

**차체 운동:**
```
M·z̈   = Σᵢ (−F_s[i] − F_d[i] + F_a[i])
I_y·θ̈ = Σᵢ x_pos[i]·(−F_s[i] − F_d[i] + F_a[i])
I_x·φ̈ = Σᵢ (−y_pos[i])·(−F_s[i] − F_d[i] + F_a[i])
```

**바퀴 운동:**
```
m_u·z̈_u[i] = F_s[i] + F_d[i] − F_a[i] − k_t·(z_u[i] − z_r[i])
```
여기서 `z_r[i]`는 그 바퀴 위치의 노면 높이.

### 적분 방식
세미 임플리시트 오일러(semi-implicit Euler), 애니메이션 1프레임당 20 substep (`dt = dt_real/20`). 타이어 강성 `k_t = 250000 N/m`이 가장 빠른 모드(~13 Hz)를 만들기 때문에 substep을 더 줄이면 진동이 발산할 수 있다.

### 물리 상수 (코드에서)
| 기호 | 값 | 의미 |
|---|---|---|
| `M` | 1200 kg | 차체 질량 |
| `I_y` | 2100 kg·m² | 피치 관성모멘트 |
| `I_x` | 600 kg·m² | 롤 관성모멘트 |
| `m_u` | 50 kg | 바퀴 1개 질량 |
| `k_s` | 32000 N/m | 서스펜션 스프링 강성 |
| `c_s` | 1100 N·s/m | 서스펜션 감쇠 (부드러운 세팅) |
| `k_t` | 250000 N/m | 타이어 강성 |
| `L_f, L_r` | 1.2, 1.4 m | 앞·뒤 휠베이스 |
| `W` | 0.78 m | 트레드 (반폭) |

---

## 3. 제어 모드 두 가지

### Mode 0 — 맥퍼슨 스트럿 (수동)
`F_a = 0`. 코일 스프링 + 텔레스코픽 댐퍼만으로 충격 흡수. 일반 SUV에서 가장 흔한 방식. 단일 댐핑 계수라 모든 주파수에 똑같이 반응해서, 큰 충격에서는 차체가 흔들리거나 진동이 오래 남는다. 이번 프로젝트에서는 비교 기준선(baseline)으로 사용.

### Mode 1 — 분수미분 능동 제어 D^α
```
F_a[i] = −K · D^α(z_c[i])
```
- α는 **0.5 / 0.6 / 0.7** 중 사용자가 선택
- α = 0이면 위치 피드백(스프링형), α = 1이면 속도 피드백(스카이훅형)
- **α가 그 중간 값이면 위치와 속도 사이를 보간**하는 "기억 효과" 피드백이 됨

### 분수미분의 수치 구현 — Grünwald–Letnikov 정의

연속 신호 f(t)의 α차 분수미분은
$$
D^α f(t) = \lim_{h \to 0} \frac{1}{h^α} \sum_{k=0}^{N} (-1)^k \binom{α}{k} f(t - kh)
$$
로 정의된다. 여기서 일반화된 이항계수는
$$
\binom{α}{k} = \frac{α(α-1)(α-2)\cdots(α-k+1)}{k!}
$$
재귀로 계산: `coeff(0) = 1`, `coeff(k+1) = coeff(k) · (k − α) / (k + 1)`.

이산 시간(시뮬레이션 한 step 길이 `dt`)에서:
```
D^α f[n] ≈ (1 / dt^α) · Σ_{k=0}^{N} c_k · f[n − k]
   where c_k = (-1)^k · C(α, k)
```

코드 구현(`fracDeriv` 함수, 단일 함수로 200개 미만의 과거 샘플만 사용해 절단 — truncation):
```javascript
function fracDeriv(hist, alpha, dt) {
  const N = Math.min(hist.length, 200);
  let sum = 0;
  let coeff = 1;
  for (let k = 0; k < N; k++) {
    sum += (k % 2 === 0 ? 1 : -1) * coeff * hist[hist.length - 1 - k];
    coeff *= (k - alpha) / (k + 1);
  }
  return sum / Math.pow(dt, alpha);
}
```

> **물리적 직관**: α차 분수미분은 "현재의 위치 변화율"과 "지금까지의 누적 이력"을 둘 다 반영한다. α를 크게 할수록 최근 변화에 더 민감하게(속도형), 작게 할수록 변화의 누적에 더 민감하게(위치형) 반응한다.

---

## 4. 도로(노면) 모델

### 평소 노면
`roadHeight(x, y) = 작은 사인파 잔잔한 굴곡` — 차량이 항상 약간 출렁이는 배경 진동.

### 장애물(bumps) 종류 — UI 버튼으로 추가
| 버튼 | 설명 | 좌·우 영향 |
|---|---|---|
| 낮은 방지턱 | 길이 0.9 m, 높이 4 cm | 양쪽 |
| 높은 방지턱 | 길이 1.3 m, 높이 13 cm | 양쪽 |
| 좌우 다른 도로 | 약간 비대칭 (h=0.09 vs 0.065) | 좌·우 따로 |
| 좌우 높이차 | 같은 위치에서 좌 12 cm / 우 5 cm | 좌·우 따로 — **롤 즉시 유발** |
| 좌2회 / 우2회 | 같은 쪽에 2.5 m 간격 두 번 | 한쪽만 |

각 버튼 클릭은 하나의 **장애물 그룹 id**(`B1`, `B2`, …)를 부여해 RMS 비교 단위가 된다.

---

## 5. 측정 — 차체 가속도 RMS

### 왜 RMS인가
승차감 평가에서는 ISO 2631 등에서 차체 진동의 RMS(root mean square)를 표준 지표로 쓴다. 순간 가속도가 아니라 일정 시간 윈도의 평균적인 진동 강도를 보여주기 때문.

### 두 가지 RMS 그래프

**(A) 실시간 RMS (롤링 ~1초 윈도)**
```
RMS(t) = sqrt( (1/N) · Σ_{i=t−N}^{t} z̈²[i] )      (N ≈ 200 샘플)
```
시간이 흐름에 따라 가장 위 그래프에 청록색 라인으로 표시. 장애물 통과 구간은 색띠로 음영 + 라벨 + 평균(avg) / 피크(pk) 표시.

**(B) 실험구간 RMS (첫 장애물 ~ 마지막 장애물)**
거리축으로 표시. 차량이 첫 장애물에 진입하는 순간 자동 기록 시작, 마지막 장애물 통과 + 1.5 m 후 자동 종료. 노란 점선으로 구간 평균 RMS, 우측 상단에 피크값을 g 단위까지 표시.

### 장애물별 비교 표
실시간 RMS 그래프 아래에 자동 생성되는 표:

| 구간 | 종류 | 평균 (m/s²) | 피크 (m/s²) | ≈ g |
|---|---|---|---|---|
| B1 | 낮은턱 | ... | ... | ... |
| B2 | 좌우경사 | ... | ... | ... |

각 모드/차수에서 같은 장애물 시퀀스를 통과시키면 표의 숫자로 모드 간 차이를 정량 비교할 수 있다.

---

## 6. 시각화 / UI

### 메인 화면 (좌측 패널)
- 1100 × 460 캔버스에 **사선 투영(oblique projection)** 으로 차량과 도로를 그림 — 원근감은 없지만 깊이감은 있는 형태
- 차체는 챠시 + 캐빈 두 박스, 바퀴 4개, 코일 스프링, 브레이크 캘리퍼까지 묘사
- 카메라는 항상 차량을 중앙에 고정해 도로/장애물이 흘러가는 것처럼 보임

### 컨트롤 (가운데 패널)
- 서스펜션 모드 토글 (수동 / 분수미분)
- 분수 차수 선택 (0.5 / 0.6 / 0.7)
- 도로 굴곡 추가 6종
- 차량 속도 슬라이더 (3~35 m/s)
- 제어 게인 K 슬라이더 (500~8000)
- 정지/재개 버튼

### 그래프 (우측 패널, 위→아래)
1. **차체 가속도 RMS · 장애물별 비교** — 메인 평가 그래프
2. **실험 구간 RMS (첫 ~ 마지막 장애물)** — 거리축 비교
3. **장애물별 평균/피크 비교 표**
4. **차체 vs 노면 응답** — 변위 시계열 (실시간 + 실험구간)
5. **실시간 상태** — 차체 변위, 피치, 롤, 차체 가속도, 진동 RMS, 탑승감 점수

---

## 7. 개발 과정 (시간순)

| 일자 | 커밋 | 변경 내용 | 이유 |
|---|---|---|---|
| 2026-06-10 00:27 | `5a77c72` | 초기 커밋 (7-DoF + 다중 차수 분수미분 + 정수 skyhook + 5종 장애물 + 정지 버튼) | 베이스라인 |
| 2026-06-10 22:50 | `e33e23b` | 차체 가속도 RMS 실시간 그래프 추가 (롤링 ~1s 윈도) | 승차감 정량 평가를 위한 기준 지표 도입 |
| 2026-06-10 22:55 | `69aefa1` | 수동 댐핑 1800→1100 약화 / 정수미분 모드 제거 / 분수 차수 선택(0.5/0.6/0.7) | 비교 단순화: 두 모드 + 차수별 미세 비교에 집중 |
| 2026-06-10 23:05 | `740cb04` | RMS 그래프를 최상단으로 + 장애물 통과 구간 음영 + 비교 표 | "그래프를 비교하고 싶다"는 실험 도구 요구 반영 |
| 2026-06-10 23:11 | `311d412` | 실험구간(첫~마지막 장애물) 가속도 RMS 그래프 추가 | 거리축 기반의 모드 간 정량 비교 강화 |

> 그 이전(2026-06-01 ~ 06-03) 약 3일에 걸쳐 별도 디렉토리 `simul01/`에서 베이스 시뮬레이터를 개발한 작업도 있음. 7-DoF 동역학, 분수미분 구현, 장애물 6종, 실험구간 변위 그래프, 정지 버튼 등이 그 시점에 완성.

---

## 8. 핵심 코드 발췌

### (a) 한 step의 물리 적분 — `physicsStep(dt)`
```javascript
function physicsStep(dt) {
  const z_c = [], z_c_dot = [];
  for (let i = 0; i < 4; i++) {
    z_c[i]     = s.z + x_pos[i] * s.theta - y_pos[i] * s.phi;
    z_c_dot[i] = s.z_dot + x_pos[i] * s.theta_dot - y_pos[i] * s.phi_dot;
  }

  const Fs = [], Fd = [], Fa = [];
  for (let i = 0; i < 4; i++) {
    Fs[i] = k_s * (z_c[i] - s.z_u[i]);
    Fd[i] = c_s * (z_c_dot[i] - s.z_u_dot[i]);
    s.hist[i].push(z_c[i]);
    if (s.hist[i].length > 220) s.hist[i].shift();

    if (mode === 0) {
      Fa[i] = 0;                                          // 맥퍼슨 수동
    } else if (mode === 1) {
      Fa[i] = -K_active * fracDeriv(s.hist[i], fracAlpha, dt); // 분수미분
    }
    // 액추에이터 한계
    if (Fa[i] > 6000) Fa[i] = 6000;
    if (Fa[i] < -6000) Fa[i] = -6000;
  }

  let zdd = 0, tdd = 0, pdd = 0;
  for (let i = 0; i < 4; i++) {
    const F_b = -Fs[i] - Fd[i] + Fa[i];
    zdd += F_b;
    tdd += x_pos[i] * F_b;
    pdd += -y_pos[i] * F_b;
  }
  zdd /= M; tdd /= I_y; pdd /= I_x;

  // 바퀴 가속도
  const zudd = [];
  for (let i = 0; i < 4; i++) {
    const xw = car_x + x_pos[i], yw = y_pos[i];
    const zr = roadHeight(xw, yw);
    zudd[i] = (Fs[i] + Fd[i] - Fa[i] - k_t * (s.z_u[i] - zr)) / m_u;
  }

  // 세미 임플리시트 오일러
  s.z_dot   += zdd * dt; s.z     += s.z_dot   * dt;
  s.theta_dot += tdd * dt; s.theta += s.theta_dot * dt;
  s.phi_dot   += pdd * dt; s.phi   += s.phi_dot   * dt;
  for (let i = 0; i < 4; i++) {
    s.z_u_dot[i] += zudd[i] * dt;
    s.z_u[i]     += s.z_u_dot[i] * dt;
  }

  car_x += carSpeed * dt;

  // RMS용 버퍼 (가속도 절댓값, 변위 가중합)
  zddBuf.push(Math.abs(zdd));
  if (zddBuf.length > 200) zddBuf.shift();
  // ...
}
```

### (b) 분수미분 — Grünwald–Letnikov (위에서 인용한 함수)

### (c) 롤링 RMS 갱신 (메인 루프 안)
```javascript
let sumSq = 0;
for (const v of zddBuf) sumSq += v * v;
const accRms = Math.sqrt(sumSq / Math.max(1, zddBuf.length));
accRmsData.push(accRms);
if (accRmsData.length > 280) accRmsData.shift();
globalSampleIdx++;
```

### (d) 장애물 통과 구간 자동 추적
```javascript
let currentBumpId = null, currentBumpLabel = '';
for (const b of bumps) {
  const overlap = (car_x + L_f >= b.x) && (car_x - L_r <= b.x + b.len + 0.6);
  if (overlap) {
    if (currentBumpId === null || b.x < bumps.find(x => x.id === currentBumpId).x) {
      currentBumpId = b.id;
      currentBumpLabel = b.typeLabel || '';
    }
  }
}

if (currentBumpId !== null) {
  if (!activeBumpPass || activeBumpPass.bumpId !== currentBumpId) {
    if (activeBumpPass) { /* 이전 pass 확정 */ }
    activeBumpPass = {
      bumpId: currentBumpId,
      label: 'B' + currentBumpId,
      startSample: globalSampleIdx,
      peak: accRms, sum: accRms, count: 1, avg: accRms,
      // ...
    };
  } else {
    if (accRms > activeBumpPass.peak) activeBumpPass.peak = accRms;
    activeBumpPass.sum += accRms;
    activeBumpPass.count++;
    activeBumpPass.avg = activeBumpPass.sum / activeBumpPass.count;
  }
} else if (activeBumpPass) {
  bumpPasses.push(activeBumpPass);
  activeBumpPass = null;
}
```

---

## 9. 실험 절차 (다른 사람이 재현 가능)

1. 라이브 데모 URL 접속
2. 속도 슬라이더 14 m/s, 제어 게인 3500으로 고정 (디폴트)
3. **수동 모드** 선택 → "낮은 방지턱", "좌우 높이차", "높은 방지턱" 순서로 클릭
4. 차량이 마지막 장애물 통과할 때까지 대기 → 실험구간 RMS 그래프 평균/피크 기록
5. "초기화" → **분수미분 모드, α=0.5** 선택 → 같은 시퀀스 반복 → 기록
6. α=0.6, α=0.7도 동일하게 반복
7. 표:
   | 모드 | 평균 RMS (m/s²) | 피크 RMS (m/s²) |
   |---|---|---|
   | 수동 | ? | ? |
   | D^0.5 | ? | ? |
   | D^0.6 | ? | ? |
   | D^0.7 | ? | ? |

---

## 10. 배운 점 (보고서 마무리에 활용)

- **수학과 공학의 연결**: 분수미분이라는 추상적 수학 개념(Grünwald–Letnikov 공식)이 자동차 승차감 같은 일상 문제에 실제로 응용된다는 것을 직접 코드로 확인.
- **시뮬레이션 검증의 어려움**: 적분 시간 step을 잘못 잡으면 안정한 시스템도 발산한다. 타이어 강성이 가장 빠른 모드를 결정한다는 것을 substep 줄이다 발견.
- **정량 평가의 중요성**: 눈으로 "부드러워 보인다"가 아니라 RMS 같은 수치 지표로 비교해야 객관적 판단 가능. 같은 장애물 시퀀스를 통과시켰을 때만 공정한 비교가 됨 → "실험구간 그래프"의 필요성.
- **UX와 실험 도구의 결합**: 사용자가 모드를 바꿔가며 같은 장애물을 다시 만들 필요 없도록 장애물 통과를 자동 감지해서 구간별 평균을 자동 측정하는 기능을 구현.

---

## 다른 Claude에게 던질 프롬프트 예시

아래 텍스트 그대로 다른 Claude(claude.ai 웹 등)에 붙여 넣으세요:

> 안녕! 나는 고등학생인데 학교 생기부에 들어갈 프로젝트 보고서를 쓰려고 해. 내가 자동차 서스펜션 시뮬레이터를 만들었어. 아래 컨텍스트 문서를 읽고, **A4 5장 정도 분량의 생기부용 보고서**를 한국어로 작성해줘. 구성은 다음 순서로:
>
> 1. 동기 (왜 이 프로젝트를 했나)
> 2. 배경 지식 (자동차 서스펜션, 분수미분 — 고등학생 수준으로)
> 3. 시뮬레이터 설계 (7자유도 모델, 두 모드, 분수미분 수치 구현)
> 4. 실험 방법 (장애물 시퀀스, 측정 지표)
> 5. 예상되는 결과 분석 양식 (실제 숫자는 비워두고 표 양식만)
> 6. 배운 점 / 진로 연계 (수학·공학적 사고력, 데이터 시각화)
> 7. 참고 자료
>
> 톤은 너무 어른스럽거나 학술적이지 않게, 고등학생이 스스로 쓴 느낌으로. 다만 분수미분 부분은 수학적 표현을 정확히 써줘.
>
> 전체 코드는 여기에서 볼 수 있어: https://raw.githubusercontent.com/iambaek/suspension-simulator/main/playground.html
>
> 라이브 데모: https://iambaek.github.io/suspension-simulator/
>
> 아래는 프로젝트 컨텍스트야:
>
> ---
>
> (여기에 이 문서 전체를 붙여넣기)

---

*생성: 2026-06-11, 작성자: 백[성] / 작업 환경 Claude Code*
