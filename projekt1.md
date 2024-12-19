# System poruszania się robota po kwadracie z obserwowaniem błędu pozycji

## Dobór tematów  

Rozpoczęcie tworzenia węzła w języku Python polegało na dobraniu subskrybowanych tematów do czytania pozycji robota:

- Temat **`/ground_truth_odom`** do odczytywania pozycji z symulowanego sensora odometrii.  
- Temat **`/mobile_base_controller/odom`** do odczytywania odometrii bazy robota w systemie.

Wykorzystywany będzie również temat:

- **`/cmd_vel_nav`** do zadawania prędkości liniowej i obrotowej.

## Sterowanie robotem

W przeciwieństwie do zadawania czasu ruchu i obrotu jak na laboratoriach nr 1, zdecydowaliśmy się na inny sposób zadawania ruchu i obrotu. 

Robot będzie wprawiany w ruch liniowy lub obrotowy, a w między czasie liczony będzie dystans pokonany przez robota, albo obrót wykonany przez robota. Gdy osięgnięta zostanie odpowiednia długość boku lub obrót o 90 stopnii, robot jest zatrzymywany.

Poniżej zaprezentowano dwie funkcje:  
1. **`move_straight()`** - do ruchu liniowego robota.  
2. **`turn()`** - do ruchu obrotowego robota.  

Poniżej znajdują się kody funkcji:

```python
    def turn(self):
        self.initial_orientation = self.current_position['theta']

        while pi/2 - abs(self.current_position['theta']-self.initial_orientation) > 0.011:
            rclpy.spin_once(self, timeout_sec=0.1)
            angular_speed = 0.3 if self.side == "left" else -0.3
            self.send_velocity(0.0, angular_speed)

            self.calculate_errors()
            self.check_temporary_errors()
        
        self.stop()

    def move_straight(self):
        distance = 0.0
        while distance < self.a - 0.05:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.send_velocity(0.2, 0.0)

            self.calculate_errors()
            self.check_temporary_errors()

            dx = self.current_position['x'] - self.initial_position['x']
            dy = self.current_position['y'] - self.initial_position['y']
            distance = sqrt(dx**2 + dy**2)

        self.stop()
```

---

## Parametry systemu

System wczytuje 3 parametry:

1. **`a`** - długość boku kwadratu.  
2. **`side`** - wybór w którą stronę będzie jechał robot.
3. **`n`** - ilość powtórzeń okrążeń.

Parametry są wczytywane wraz z wywołąniem węzła w terminalu. Tak prezentuje się przykładowe wywołanie systemu:
![alt text](projekt1/foty/command_kwadrat.png)

## Działanie ruchu

Poniżej zamieszczono rezultat zadanego ruchu robota w wizualizacji z programu Rviz:
![alt text](projekt1/foty/kwadrat.png)

## Obserwowanie błędu odometrii