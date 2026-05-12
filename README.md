# Projecte Final GIA-IR

## L'Explorador Cartògraf

Sistema d'exploració autònoma amb ROS2 Jazzy per a TurtleBot3 Burger.
El robot explora lliurement. Quan troba un obstacle, decideix quin costat té més espai
lliure per girar. Registra les coordenades dels obstacles trobats

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
- `/objecte_detectat` - posició de l'objecte en coordenades del mapa

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

- `/cmd_vel` - velocitat del robot
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
| `1`    | Maniobra paret - girar 150° cap al costat lliure | ✅              |
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

| Fitxer         | Paràmetre              | Valor actual | Descripció                             |
| -------------- | ---------------------- | ------------ | -------------------------------------- |
| `deteccio.py`  | `llindar` (normal)     | `0.30m`      | Distància de detecció d'obstacle       |
| `deteccio.py`  | `llindar` (en esquiva) | `0.25m`      | Distància de detecció en esquiva       |
| `deteccio.py`  | `llindar_centre`       | `0.40m`      | Llindar zona ±20° per classificar      |
| `deteccio.py`  | `llindar_anell`        | `0.60m`      | Llindar zona ±20°–±60° per classificar |
| `cartograf.py` | `radi_proximitat`      | `0.35m`      | Radi per filtrar duplicats             |
| `moviment.py`  | `5 * math.pi / 6`      | `150°`       | Angle de maniobra de paret             |
| `moviment.py`  | `math.pi / 2`          | `90°`        | Angle d'esquiva d'objecte              |
| `moviment.py`  | `cicles < 12`          | `1.2s`       | Temps d'avanç lateral esquiva          |
| `moviment.py`  | `cicles < 25`          | `2.5s`       | Temps d'avanç per superar objecte      |

---

## Canvis respecte a la versió original

Molts comentaris han sigut netejats.

### `deteccio.py`

- **Distància de detecció** `0.25m` en exploració i `0.20m` en esquiva (con reduït a ±20°) - evita reaccions prematures
- **Classificació per zones** substitueix el llindar de `> 90 punts`: zona centre (±20°, `llindar_centre = 0.40m`) i zona anell (±20°–±60°, `llindar_anell = 0.60m`). PARET si ambdues zones superen el 50%; OBJECTE si només ho fa el centre
- **`en_maniobra`** ara té 3 valors: `0` (explorant, con ±60°, llindar `0.30m`), `1` (girant, detecció OFF), `2` (avançant en esquiva, con reduït a ±20°, llindar `0.25m`)

### `cartograf.py`

- **`radi_proximitat`** augmentat de `0.2m` a `0.35m` - absorbeix el soroll d'odometria sense confondre objectes propers

### `moviment.py`

- **Odometria per als girs** - afegida subscripció a `/odom` i funcions `iniciar_gir()` i `angle_girat()`. Els girs ara s'aturen quan es mesura l'angle real, no per cicles fixos
- **Maniobra paret** - un sol gir de 150° cap al costat lliure (estat 1), elimina l'antic estat 2 de 90°
- **Llindar incrementat** - aprofitant que `moviment.py` també llegeix de `/scan`, augmentar el rang a 180°, només quan ha de fer una decisió; no per detectar nous objectes.
- **Costat lliure automàtic** - nova funció `calcular_costat_lliure()` que compta els raigs vàlids a esquerra i dreta del làser i tria el costat amb més espai. S'usa tant per paret com per objecte
- **Lògica dreta/esquerra corregida** - la versió original girava cap al costat incorrecte
- **`en_maniobra`** publicat com `1` durant girs (1, 10, 12, 14, 16), `2` durant avanços d'esquiva (11, 13, 15) i `0` en exploració

---

## Problemes coneguts

- **Girs de 90°** - cal verificar que l'odometria és prou precisa al robot real. Si no gira exactament 90°, ajustar la velocitat angular (`0.5 rad/s`) o revisar la calibració de l'odometria.
- **Objectes molt propers entre si** - si dos objectes estan a menys de `0.35m`, el filtre de `radi_proximitat` pot confondre'ls com un de sol. Reduir el radi si cal.
- **Parets en diagonal** - si el robot es troba la paret en un angle agut/obtús, és probable que la detecti com a objecte.
