# Управление в сценарии `linear_chain2`

Документ описывает сценарий `scenarios/linear_chain2.py`: **линейная цепь** с двумя якорями и **внутренним управлением**, совмещающим геометрическую стабилизацию (как в `linear_chain`) и **распределённый закон** из ветки Anatoliy / `corridor_sweeping` (четыре полупространства, нелинейность \(\tanh\), виртуальный зазор поперёк).

## Идея сценария

- **Крайние дроны** (`min(id)`, `max(id)`) — якоря: летят к фиксированным точкам \(A\) и \(B\) на отрезке длины \(L\) в плоскости \(XY\) (common frame).
- **Внутренние дроны** получают команды **без явной 2D-цели «точка на линии»**: вектор желаемого смещения в NED собирается из
  1. **геометрии цепи** — ошибки равномерности вдоль отрезка и удержания на прямой \(AB\);
  2. **закона коридора (Anatoliy)** — скаляры \(\sigma_a\), \(\sigma_c\) по ближайшим соседям в четырёх квадрантах осей \((\text{along}, \text{cross})\) в базисе отрезка.

Управление — **RC override** (roll/pitch), с переводом из NED в корпус по текущему yaw из attitude.

## Быстрый запуск

```bash
source ../drone_env/bin/activate
python Drone_Swarm_Simulator_v2/scenarios/linear_chain2.py --drones 6 --segment-length 8 --exchange-hz 50
```

Дополнительные параметры, влияющие на внутренний закон: `--r-vis` (радиус видимости соседей), `--w-cross` (виртуальный зазор поперёк при отсутствии соседа).

## Потоки и циклы

Как в `linear_chain`:

- инициализация дронов в отдельных потоках (`initialize_drone_parallel`);
- **обмен координатами** — `coordinate_exchange_loop` (частота `--exchange-hz`);
- по одному потоку на дрон:
  - якоря — `endpoint_loop`;
  - внутренние — `internal_chain_anatoliy_loop`.

Частота контура внутреннего/якорного управления: `CONTROL_HZ` (в коде 18 Гц).

## Общий кадр (common frame)

Как в `linear_chain`: локальная позиция дрона сдвигается по \(Y\):

\[
y_{\text{common}} = y_{\text{local}} + (id - 1)\cdot 2\text{ м}.
\]

Функции: `_did_offset_y`, `_my_position_common`.

## Отрезок \(AB\) и локальный базис

После взлёта якоря задают геометрию. В отличие от описания в `linear_chain_control.md` для v1, здесь **оба конца по \(Y\) совпадают** со **средним** между якорями, чтобы прямая не наклонялась из‑за расхождения \(y\) левого/правого после SITL:

\[
y_{\text{mid}} = \frac{y_{\text{left}} + y_{\text{right}}}{2},\quad
A = \bigl(-\tfrac{L}{2},\, y_{\text{mid}}\bigr),\quad
B = \bigl(+\tfrac{L}{2},\, y_{\text{mid}}\bigr)
\]

(в коде также задаётся \(z = -\texttt{TAKEOFF\_ALT\_M}\).)

**Единичный вектор вдоль цепи** и **левый перпендикуляр** в плоскости \(XY\) (NED: \(x\) — North, \(y\) — East):

\[
\mathbf{u} = \frac{B_{xy} - A_{xy}}{\lVert B_{xy} - A_{xy}\rVert},\qquad
\mathbf{n} = (-u_y,\, u_x)
\]

(поворот \(\mathbf{u}\) на \(+90^\circ\) в плоскости \(XY\): «слева» от направления цепи.)

**Проекция** точки \(p\) на ось от \(A\) вдоль \(\mathbf{u}\):

\[
s(p) = (p_{xy} - A_{xy}) \cdot \mathbf{u}.
\]

## Якоря: `endpoint_loop` и PID в корпусе

Цель якоря — точка \(A\) или \(B\) в common frame. Ошибка в NED:

\[
e_N = x_{\text{tgt}} - x_{\text{me}},\quad e_E = y_{\text{tgt}} - y_{\text{me}},
\]

с deadband `ERROR_DEADBAND_M`. Далее ошибка переводится в **вперёд / вправо** по корпусу (yaw \(\psi\) из attitude с множителем `YAW_SIGN`):

\[
e_f = e_N \cos\psi + e_E \sin\psi,\qquad
e_r = -e_N \sin\psi + e_E \cos\psi
\]

(реализация: `_ned_to_body_forward_right`.)

Два PID (в коде \(K_i=K_d=0\)):

\[
u_{\text{pitch}} = \text{PID}_{\text{pitch}}(e_f),\quad
u_{\text{roll}} = \text{PID}_{\text{roll}}(e_r),
\]

ограничение выхода `ENDPOINT_OUTPUT_LIMIT`, затем

\[
\text{pitch}_{\text{PWM}} = \texttt{RC\_NEUTRAL} - u_{\text{pitch}},\quad
\text{roll}_{\text{PWM}} = \texttt{RC\_NEUTRAL} + u_{\text{roll}}
\]

(и clamp, опционально обмен каналов `RC_OVERRIDE_SWAP_ROLL_PITCH`).

После достижения радиуса `ENDPOINT_REACHED_THRESH_M` — удержание нейтральным RC.

## Внутренние дроны: `internal_chain_anatoliy_loop`

### Предпосылки и ожидание данных

Собирается множество позиций \(\{p_i\}\) в common frame (своя + чужие из обмена). Если **нет** позиций обоих якорей или **меньше двух** видимых соседей (см. ниже) — нейтральный RC.

### Соседи в «сенсорной» модели: оси along / cross

Относительно своей позиции \(p_{\text{me}}\) для каждого другого дрона \(j\) (в радиусе `R_VIS_M`):

\[
\Delta_{xy} = p_{j,xy} - p_{\text{me},xy},\quad
a_j = \Delta_{xy}\cdot \mathbf{u},\quad
c_j = \Delta_{xy}\cdot \mathbf{n}.
\]

### Четыре квадранта и расстояния \(d_{a+}, d_{c+}, d_{a-}, d_{c-}\)

Для всех пар \((a_j, c_j)\) находятся минимумы:

- среди \(a_j \ge 0\) — минимальное \(a_j\) \(\Rightarrow d_{a+}\) (флаг «есть сосед»);
- среди \(a_j < 0\) — минимальное \(|a_j|\) \(\Rightarrow d_{a-}\);
- аналогично для \(c_j \ge 0\) и \(c_j < 0\) \(\Rightarrow d_{c+}, d_{c-}\).

Если в каком‑то направлении соседей нет, соответствующее расстояние остаётся «бесконечным», а в формулах ниже подставляется **виртуальное** значение через функцию \(g\) (см. следующий подраздел).

### Нелинейность и функция \(g\)

\[
\xi(\cdot) = \tanh(\cdot).
\]

\[
g(d,\, w,\, \text{has\_peer}) =
\begin{cases}
d, & \text{если сосед есть},\\
w, & \text{если соседа нет}.
\end{cases}
\]

В коде: по оси **вдоль** для отсутствующего направления используется \(w=0\); по оси **поперёк** — \(w = \texttt{W\_CROSS\_M}\).

### Скорости вдоль/поперёк и сигмы \(\sigma_a\), \(\sigma_c\)

В полной постановке коридора в \(\sigma\) входят проекции скорости \(v_{\text{along}}\), \(v_{\text{cross}}\). В этом сценарии для устойчивости в SITL **принудительно** \(v_{\text{along}} = v_{\text{cross}} = 0\) (см. комментарий в коде).

Тогда:

\[
\sigma_a = -\xi\bigl(g(d_{a+}, 0, f_{a+})\bigr) + \xi\bigl(g(d_{a-}, 0, f_{a-})\bigr),
\]

\[
\sigma_c = -\xi\bigl(g(d_{c+}, w_{\text{cross}}, f_{c+})\bigr) + \xi\bigl(g(d_{c-}, w_{\text{cross}}, f_{c-})\bigr),
\]

где \(f_{\ast}\) — булевы флаги наличия соседа в соответствующем полупространстве.

### Геометрические ошибки \(e_{\text{along}}\), \(e_{\text{cross}}\)

Все дроны упорядочиваются по \(s(p_i)\) (при равном \(s\) — по `id`). Для **внутреннего** дрона с индексами соседей слева/справа по этому порядку:

\[
e_{\text{along}} = \frac{s_L + s_R}{2} - s_{\text{me}},
\]

если есть оба соседа; иначе \(e_{\text{along}} = 0\).

**Поперечная** (signed) ошибка относительно прямой через \(A\) вдоль \(\mathbf{u}\):

\[
e_{\text{cross}} = (p_{\text{me},xy} - A_{xy})\cdot \mathbf{n}.
\]

Ошибки ограничиваются по модулю (`SPACING_E_ALONG_CLAMP_M`, `SPACING_E_CROSS_CLAMP_M`).

### Вектор в NED и мёртвые зоны

Геометрическая тяга (как в комментарии к коду: вдоль \(\mathbf{u}\) от \(e_{\text{along}}\), против \(\mathbf{n}\) от \(e_{\text{cross}}\)):

\[
\mathbf{g}_{xy} = k_{\text{along}}\, e_{\text{along}}\, \mathbf{u} - k_{\text{cross}}\, e_{\text{cross}}\, \mathbf{n},
\]

где \(k_{\text{along}} = \texttt{SPACING\_ALONG\_K}\), \(k_{\text{cross}} = \texttt{SPACING\_CROSS\_K}\).

Вклад Anatoliy в плоскости \(XY\):

\[
\mathbf{a}_{xy} = \sigma_a\, \mathbf{u} + \sigma_c\, \mathbf{n}.
\]

**Итоговый** горизонтальный вектор комбинации (в коде компоненты \(x,y\) — это North/East):

\[
\mathbf{c}_{xy} = \mathbf{g}_{xy} + w_{\text{Anat}}\, \mathbf{a}_{xy},\quad
w_{\text{Anat}} = \texttt{ANATOLIY\_COMB\_WEIGHT}.
\]

Если одновременно малы \(\sigma_a,\sigma_c\) (ниже `SIGMA_DEADBAND`) **и** нет «геометрического натяжения» вне deadband `SPACING_E_DEADBAND_M` — выдаётся нейтральный RC.

### Демпфирование по скорости и перевод в стики

Сначала вектор \(\mathbf{c}_{xy} = (c_N, c_E)^\top\) (North/East) переводится в корпус **вперёд/вправо** той же ротацией \(\psi\), что и для якоря:

\[
\begin{pmatrix} f_0 \\ r_0 \end{pmatrix}
=
\begin{pmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{pmatrix}
\begin{pmatrix} c_N \\ c_E \end{pmatrix}.
\]

Горизонтальная скорость в NED \((v_x, v_y)\) переводится в корпус \((v_f, v_r)\) той же матрицей. Демпфирование (с ограничением по модулю `INTERNAL_BODY_V_DAMP_CLAMP`):

\[
f = f_0 - \operatorname{clamp}\bigl(k_d\, v_f\bigr),\quad
r = r_0 - \operatorname{clamp}\bigl(k_d\, v_r\bigr),
\]

где \(k_d = \texttt{INTERNAL\_BODY\_V\_DAMP}\), а \(\operatorname{clamp}\) — насыщение по `INTERNAL_BODY_V_DAMP_CLAMP`.

Далее **пропорциональный закон** по осям корпуса с ограничением PWM, **минимальный шаг** `INTERNAL_RC_MIN_STEP_PWM` при ненулевой ошибке и нулевом округлении, **slew-rate** за такт `INTERNAL_RC_SLEW_PWM_PER_CYCLE`. Для roll коэффициент умножается на `INTERNAL_RC_KP_ROLL_MULT`.

Итоговые PWM снова маппятся как у якорей: pitch от «вперёд», roll от «вправо», с тем же `RC_OVERRIDE_SWAP_ROLL_PITCH`.

## Сводка параметров (где смотреть в коде)

В начале `linear_chain2.py`:

| Группа | Константы |
|--------|-----------|
| Геометрия | `TAKEOFF_ALT_M`, `SEGMENT_LENGTH_M` |
| Якоря / PID | `ENDPOINT_*`, `ERROR_DEADBAND_M`, `PID_KI`, `PID_KD` |
| Частоты | `CONTROL_HZ`, аргумент `--exchange-hz` |
| Коридор / соседи | `R_VIS_M`, `W_CROSS_M`, `SIGMA_DEADBAND`, `ANATOLIY_COMB_WEIGHT` |
| Геометрия цепи | `SPACING_ALONG_K`, `SPACING_CROSS_K`, `SPACING_E_*` |
| Внутренний RC | `INTERNAL_RC_*`, `INTERNAL_BODY_V_DAMP*` |
| Attitude / RCMAP | `YAW_SIGN`, `INTERNAL_RC_KP_ROLL_MULT`, `RC_OVERRIDE_SWAP_ROLL_PITCH` |

## Связь с `linear_chain` и `linear_chain_control.md`

- **Общее**: два якоря, отрезок \(AB\), обмен координатами, RC override, common frame со сдвигом по \(Y\).
- **Отличия v2**: среднее \(y_{\text{mid}}\) для \(A,B\); внутренние не строят сглаженную целевую точку и один PID к ней — вместо этого **вектор \(\mathbf{c}_{xy}\)** из геометрии + \(\sigma\) закона коридора, затем прямой P‑закон по стикам с slew и bang-bang минимумом; якорный PID использует **поворот ошибки в корпус** перед регулятором.

Для пошаговой логики v1 (соседи по \(s\), сглаживание цели, демпфирование через сдвиг \(s_{\text{tgt}}\)) см. `scenarios/linear_chain_control.md`.
