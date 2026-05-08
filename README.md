# Projecte Final GIA-IR

## L'Explorador Cartògraf

Sistema d'exploració autònoma amb ROS2 Jazzy per a TurtleBot3 Burger.
El robot explora lliurement. Quan troba un obstacle, decideix quin costat té més espai
lliure per girar. Registra les coordenades dels obstacles trobats.

---

## Requisits

- ROS2 Jazzy
- TurtleBot3 Burger

---

## Variables d'entorn

Cal tenir aquestes variables configurades en cada terminal, preferiblement a `.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Instal·lació

```bash
cd ~/ros2_ws
git clone https://github.com/joseprodriguez-rv/robotica_practica/tree/gael src
```

## Compilació

Sempre des de `ros2_ws`, no des de `src`:

```bash
cd ~/ros2_ws
colcon build --packages-select projecte_final_pkg
source install/setup.bash
```

## Llançament

Requereix una terminal paral·lela oberta amb Gazebo o bé el robot real.

```bash
ros2 launch projecte_final_pkg projecte.launch.py
```

---

## Nodes

### `deteccio.py`

Llegeix el làser i l'odometria. Classifica obstacles i publica la seva posició.

**Topics subscrits:**

- `/scan` - lectura del làser
- `/odom` - posició i orientació del robot
- `/en_maniobra` - flag per pausar detecció durant girs
- `/comptador_objectes` - per parar quan s'arriba a 5 objectes

**Topics publicats:**

- `/tipus_obstacle` - `'PARET'` o `'OBJECTE'`
- `/objecte_detectat` - posició de l'objecte en coordenades del mapa (missatge `Odometry`)

---

### `cartograf.py`

Rep les posicions dels objectes detectats i manté un mapa filtrant duplicats.

**Topics subscrits:**

- `/objecte_detectat` - posició de l'objecte

**Topics publicats:**

- `/comptador_objectes` - nombre d'objectes únics registrats

---

### `moviment.py`

Controla el moviment del robot. Gestiona l'exploració, la maniobra de paret i l'esquiva d'objectes.

**Topics subscrits:**

- `/scan` - làser per decidir costat lliure
- `/odom` - per mesurar angles amb odometria
- `/comptador_objectes` - per parar en arribar a 5
- `/tipus_obstacle` - per reaccionar a obstacles

**Topics publicats:**

- `/cmd_vel` - velocitat del robot (`TwistStamped`)
- `/en_maniobra` - flag per pausar detecció durant girs

---

## Flux de dades entre nodes

```mermaid
graph TD
    scan[/scan/] --> deteccio
    scan --> moviment
    odom[/odom/] --> deteccio
    odom --> moviment
    en_maniobra[/en_maniobra/] --> deteccio
    comptador[/comptador_objectes/] --> deteccio
    comptador --> moviment

    deteccio --> tipus_obstacle[/tipus_obstacle/]
    deteccio --> objecte_detectat[/objecte_detectat/]

    tipus_obstacle --> moviment
    objecte_detectat --> cartograf

    cartograf --> comptador
    moviment --> cmd_vel[/cmd_vel/]
    moviment --> en_maniobra

    cmd_vel --> robot((TurtleBot3))
```

---

## Estats del moviment

| Estat  | Descripció                                       | Detecció activa |
| ------ | ------------------------------------------------ | --------------- |
| `0`    | Explorar en línia recta                          | ✅              |
| `1`    | Maniobra paret - gir 150° cap al costat lliure   | ❌              |
| `None` | Objectiu complert - robot aturat                 | ❌              |
| `10`   | Esquiva objecte - gir 90° cap al costat lliure   | ❌              |
| `11`   | Esquiva objecte - avançar lateral                | ✅              |
| `12`   | Esquiva objecte - gir 90° cap al costat contrari | ❌              |
| `13`   | Esquiva objecte - avançar per superar l'objecte  | ✅              |
| `14`   | Esquiva objecte - gir 90° cap al costat contrari | ❌              |
| `15`   | Esquiva objecte - avançar per tornar a la ruta   | ✅              |
| `16`   | Esquiva objecte - gir 90° per redreçar-se        | ❌              |

---

## Paràmetres ajustables

| Fitxer         | Paràmetre             | Valor actual | Descripció                                             |
| -------------- | --------------------- | ------------ | ------------------------------------------------------ |
| `deteccio.py`  | `llindar` (explorant) | `0.25m`      | Distància de detecció en exploració (`en_maniobra==0`) |
| `deteccio.py`  | `llindar` (esquiva)   | `0.20m`      | Distància de detecció en esquiva (`en_maniobra==2`)    |
| `deteccio.py`  | `llindar_centre`      | `0.40m`      | Llindar zona central ±20° per classificar              |
| `deteccio.py`  | `llindar_anell`       | `0.60m`      | Llindar zona anell ±20°–±60° per classificar           |
| `cartograf.py` | `radi_proximitat`     | `0.35m`      | Radi per filtrar duplicats                             |
| `moviment.py`  | `5 * math.pi / 6`     | `150°`       | Angle de maniobra de paret (estat 1)                   |
| `moviment.py`  | `math.pi / 2`         | `90°`        | Angle d'esquiva d'objecte                              |
| `moviment.py`  | `cicles < 12`         | `1.2s`       | Temps d'avanç lateral esquiva (estats 11 i 15)         |
| `moviment.py`  | `cicles < 25`         | `2.5s`       | Temps d'avanç per superar objecte (estat 13)           |

---

## Lògica de classificació d'obstacles (`deteccio.py`)

La classificació es fa sobre el con frontal (±60° explorant, ±20° en esquiva). S'analitzen dues zones angulars:

- **Centre** (`ranges[0:20]` + `ranges[340:360]`, ±20°): llindar `0.40m`. Raonament geomètric: una paret a `0.25m` perpendicular fa que els rajos oblics arribin fins a `0.25/cos(20°) ≈ 0.27m`; es dona marge fins a `0.40m`.
- **Anell** (`ranges[20:60]` + `ranges[300:340]`, ±20°–±60°): llindar `0.60m`. Una paret a `0.25m` perpendicular fa que els rajos a 60° arribin fins a `0.25/cos(60°) = 0.50m`; es dona marge fins a `0.60m`.

Criteris (sobre la proporció de punts vàlids per sota del llindar):

| Condició                 | Classificació |
| ------------------------ | ------------- |
| centre >50% i anell >50% | `PARET`       |
| centre >50% i anell ≤50% | `OBJECTE`     |
| qualsevol altre cas      | _(res)_       |

---

## Canvis respecte a la versió original

Molts comentaris han sigut netejats.

### `deteccio.py`

- **Distància de detecció** `0.25m` en exploració i `0.20m` en esquiva (con reduït a ±20°) - evita reaccions prematures
- **Classificació per zones** substitueix el llindar de `> 90 punts`: zona centre (±20°, `llindar_centre = 0.40m`) i zona anell (±20°–±60°, `llindar_anell = 0.60m`). PARET si ambdues zones superen el 50%; OBJECTE si només ho fa el centre. Els llindars es justifiquen geomètricament (veure secció anterior)
- **`en_maniobra`** té 3 valors: `0` (explorant, con ±60°, llindar `0.25m`), `1` (girant, detecció OFF), `2` (avançant en esquiva, con reduït a ±20°, llindar `0.20m`)

### `cartograf.py`

- **`radi_proximitat`** augmentat de `0.2m` a `0.35m` - absorbeix el soroll d'odometria sense confondre objectes propers

### `moviment.py`

- **Odometria per als girs** - afegida subscripció a `/odom` i funcions `iniciar_gir()` i `angle_girat()`. Els girs s'aturen quan es mesura l'angle real, no per cicles fixos
- **Maniobra paret** - un sol gir de 150° cap al costat lliure (estat 1)
- **`calcular_costat_lliure()`** - compta els raigs vàlids als 90° esquerra (`ranges[0:90]`) i dreta (`ranges[270:360]`) i tria el costat amb més espai. S'usa tant per paret com per objecte
- **Lògica dreta/esquerra corregida** - la versió original girava cap al costat incorrecte
- **`en_maniobra`** publicat com `1` durant girs (1, 10, 12, 14, 16), `2` durant avanços d'esquiva (11, 13, 15) i `0` en exploració
- **`TwistStamped`** en lloc de `Twist` per a `/cmd_vel`

---

## Problemes coneguts

- **Girs de 90°** - cal verificar que l'odometria és prou precisa al robot real. Si no gira exactament 90°, ajustar la velocitat angular (`0.5 rad/s`) o revisar la calibració de l'odometria.
- **Objectes molt propers entre si** - si dos objectes estan a menys de `0.35m`, el filtre de `radi_proximitat` pot confondre'ls com un de sol. Reduir el radi si cal.
- **Parets en diagonal** - si la paret es troba en un angle agut/obtús, és molt probable que sigui detectada com un objecte.
