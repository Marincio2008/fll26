from hub import port, motion_sensor
import runloop, motor_pair, motor
import color_sensor, distance_sensor

#da questa linea fino alla riga 255 (def main….) sono solo definizione della classe e dei suoi metodi che NON dovete modificare, copia incollate nel vostro programma

class RobotController:
    """
    SISTEMA DI CONTROLLO INTEGRATO PER LEGO SPIKE PRIME.
    Questa classe implementa un'architettura a oggetti per gestire la navigazione,
    la sicurezza meccanica e la percezione sensoriale del robot.
    """



    def __init__(self, left_port, right_port, color_port=None, distance_port=None, accessory_port=None, wheel_circ=17.5, track=11.2):
        """
        COSTRUTTORE: Configura l'hardware e i parametri del modello cinematico.
        :param wheel_circ: Circonferenza della ruota in cm (necessaria per calcolare gli spostamenti).
        :param track: Carreggiata (distanza tra le ruote) in cm.
        """
        self.left_port = left_port
        self.right_port = right_port
        self.color_port = color_port
        self.distance_port = distance_port
        self.accessory_port = accessory_port
        self.PAIR = motor_pair.PAIR_1

        # --- PARAMETRI FISICI ---
        self.WHEEL_CIRCUMFERENCE = wheel_circ
        self.ROBOT_TRACK = track

        # --- PARAMETRI DI CONTROLLO (GUADAGNI) ---
        # kp_gyro: Correzione proporzionale per mantenere la direzione (imbardata).
        self.kp_gyro = 2.2
        # deg_tol: Margine di errore accettabile per le rotazioni.
        self.deg_tol = 0.4
        # kp_line: Determina la reattività del robot nel seguire il bordo di una linea.
        self.kp_line = 1.2
        # target_reflection: Valore di riferimento (setpoint) per il line follower.
        self.target_reflection = 50

        # --- STATO DI SICUREZZA ---
        # safety_trip: Flag booleano per segnalare interruzioni di emergenza.
        self.safety_trip = False

        # Inizializzazione della coppia motrice sui porti specificati
        motor_pair.pair(self.PAIR, self.left_port, self.right_port)

    # --- SISTEMA DI NAVIGAZIONE INERZIALE (IMU) ---

    def get_yaw(self):
        """
        Calcola l'angolo di imbardata (yaw).
        Nota: Il valore grezzo viene moltiplicato per -0.1 per ottenere gradi decimali coerenti.
        """
        return -0.1 * motion_sensor.tilt_angles()[0]

    def normalize_angle(self, angle):
        """
        Algoritmo di normalizzazione per mantenere l'angolo nel range [-180, 179].
        Previene errori di overflow quando il robot attraversa lo zero goniometrico.
        """
        return round((angle + 180.0) % 360.0 - 180.0, 1)

    async def reset_gyro(self):
        """
        Procedura di calibrazione: azzera l'imbardata e attende che l'IMU sia stabile
        per evitare deriva (drift) termica o meccanica.
        """
        motion_sensor.reset_yaw(0)
        await runloop.until(motion_sensor.stable, timeout=3000)

    # --- GESTIONE SICUREZZA E ACCESSORI ---

    def check_accessory_stall(self):
        """
        Analisi dinamica dello stallo: verifica se la velocità angolare è nulla
        mentre il motore dovrebbe essere in movimento.
        """
        if self.accessory_port is None: return False
        return motor.velocity(self.accessory_port) == 0

    async def accessory_move_degrees(self, degrees, velocity=200):
        """Comando di attuazione per il motore accessorio (es. un braccio robotico)."""
        if self.accessory_port is None: return
        await motor.run_for_degrees(self.accessory_port, degrees, velocity)

    async def turn(self, degrees, velocity=200, mode="spin"):
        """
        Esegue una rotazione controllata.
        'spin': le ruote girano in direzioni opposte (raggio zero).
        'pivot': una ruota è ferma e l'altra gira.
        """
        steer = 100 if mode == "spin" else 50
        target_angle = self.normalize_angle(self.get_yaw() + degrees)
        steering_val = steer if degrees >= 0 else -steer

        motor_pair.move(self.PAIR, steering_val, velocity=velocity)
        # Attesa attiva fino al raggiungimento del target (feedback loop)
        await runloop.until(lambda: abs(self.normalize_angle(self.get_yaw() - target_angle)) < self.deg_tol)
        motor_pair.stop(self.PAIR)

    async def drive_straight_safe(self, distance_cm, velocity=200, target_angle=0, is_global=False, acc_degrees=0, acc_velocity=0):
            """
            Versione corretta: muove ruote e braccio insieme gestendo manualmente i gradi.
            Risolve l'errore di accessibilità verificando la presenza del motore.
            """
            if distance_cm == 0: return

            # 1. Setup Ruote
            target_motor_deg = round((abs(distance_cm) / self.WHEEL_CIRCUMFERENCE) * 360)
            actual_velocity = velocity if distance_cm > 0 else -velocity
            ref_angle = self.normalize_angle(target_angle if is_global else (self.get_yaw() + target_angle))
            start_pos_wheels = motor.relative_position(self.left_port)

            # 2. Setup Braccio (Solo se la porta è definita e il motore è collegato)
            acc_done = True
            acc_start_pos = 0

            if self.accessory_port is not None:
                try:
                    # Leggiamo la posizione iniziale SOLO se il motore risponde
                    acc_start_pos = motor.relative_position(self.accessory_port)
                    if acc_degrees != 0:
                        dir_acc = 1 if acc_degrees > 0 else -1
                        # Avviamo il motore in modalità "run" (non bloccante)
                        motor.run(self.accessory_port, abs(acc_velocity) * dir_acc)
                        acc_done = False
                except Exception:
                    # Se il motore non è collegato fisicamente, ignoriamo il braccio
                    print("Avviso: Motore accessorio non rilevato sulla porta.")
                    acc_done = True

            # 3. Variabili di controllo
            integral = 0
            self.ki_gyro = 0.05
            travelled = 0

            # --- CICLO UNIFICATO ---
            while travelled < target_motor_deg:
                # A. Controllo posizione Braccio
                if not acc_done:
                    try:
                        current_acc_pos = motor.relative_position(self.accessory_port)
                        # Se abbiamo raggiunto i gradi bersaglio, fermiamo il braccio
                        if abs(current_acc_pos - acc_start_pos) >= abs(acc_degrees):
                            motor.stop(self.accessory_port, stop=motor.BRAKE)
                            acc_done = True
                        
                    except:
                        acc_done = True # Interrompiamo il controllo se il motore viene staccato
                        

                # B. Navigazione con Giroscopio (PI Control)
                error = self.normalize_angle(self.get_yaw() - ref_angle)
                integral += error
                correction = round(self.kp_gyro * error + self.ki_gyro * integral)

                # Applichiamo la correzione in base alla direzione
                steering = -correction if actual_velocity > 0 else correction
                motor_pair.move(self.PAIR, steering, velocity=actual_velocity)

                # C. Aggiornamento distanza percorsa
                travelled = abs(motor.relative_position(self.left_port) - start_pos_wheels)
                await runloop.sleep_ms(10)

            # 4. Stop Finale per tutto
            motor_pair.stop(self.PAIR, stop=motor.SMART_BRAKE)
            if self.accessory_port is not None:
                try: motor.stop(self.accessory_port, stop=motor.BRAKE)
                except: pass

            return True

    # --- SISTEMA DI PERCEZIONE (COLORE E LINE FOLLOWING) ---

    def get_reflection(self):
        """Restituisce la luce riflessa (0-100%). Default 50 se sensore assente."""
        if self.color_port is None: return 50
        return color_sensor.reflection(self.color_port)

    async def line_follow_distance(self, distance_cm, velocity=150, side="left"):
        """
        Line Follower Proporzionale (P-Controller).
        Mantiene il sensore sul bordo della linea nera (punto di equilibrio 50% riflessione).
        """
        if self.color_port is None: return

        target_motor_deg = round((abs(distance_cm) / self.WHEEL_CIRCUMFERENCE) * 360)
        start_pos = motor.relative_position(self.left_port)
        multiplier = 1 if side == "left" else -1

        travelled = 0
        while travelled < target_motor_deg:
            # Calcolo dello steering basato sull'errore di riflessione
            error = self.target_reflection - self.get_reflection()
            steering = round(error * self.kp_line * multiplier)

            # Saturazione del segnale di controllo (clipping tra -100 e 100)
            steering = max(min(steering, 100), -100)

            motor_pair.move(self.PAIR, steering, velocity=velocity)
            travelled = abs(motor.relative_position(self.left_port) - start_pos)
            await runloop.sleep_ms(10)

        motor_pair.stop(self.PAIR)

    async def drive_until_color(self, velocity, target_color):
        """
        Avanza finché il sensore di colore non rileva l'ID colore specificato.
        Codici Colore Principali:
            0: Nero (Black)
            1: Magenta (Magenta)
            3: Blu (Blue)
            6: Verde (Green)
            7: Giallo (Yellow)
            9: Rosso (Red)
            10: Bianco (White)
            -1: Nessun colore rilevato (None/Unknown)
        """
        if self.color_port is None: return
        ref_angle = self.get_yaw()
        motor_pair.move(self.PAIR, 0, velocity=velocity)
        await runloop.sleep_ms(100) # Delay per evitare letture sporche iniziali

        while color_sensor.color(self.color_port) != target_color:
            # Mantenimento traiettoria dritta durante la ricerca
            error = self.normalize_angle(self.get_yaw() - ref_angle)
            correction = round(self.kp_gyro * error)
            motor_pair.move(self.PAIR, -correction, velocity=velocity)
            await runloop.sleep_ms(10)
        motor_pair.stop(self.PAIR)

    # --- SISTEMA DI PERCEZIONE SPAZIALE (ULTRASUONI) ---

    def get_distance_cm(self):
        """Restituisce la distanza in cm. Gestisce i valori None."""
        if self.distance_port is None: return 999
        dist = distance_sensor.distance(self.distance_port)
        # Se dist è None (fuori portata) o 0 (errore sensore), restituiamo 999
        if dist is None or dist <= 0: return 999
        return dist / 10

    async def drive_until_distance(self, target_distance_cm, velocity=200, consecutive_required=3):
        """
        Avanza dritto finché non rileva un ostacolo.
        Implementa un filtro di verifica: richiede 'consecutive_required' letture
        sotto la soglia per confermare la presenza dell'ostacolo.
        """
        if self.distance_port is None: return

        ref_angle = self.get_yaw()
        motor_pair.move(self.PAIR, 0, velocity=velocity)

        # Contatore per il filtraggio del rumore
        hits = 0

        while hits < consecutive_required:
            current_dist = self.get_distance_cm()

            # Se la distanza è valida e sotto la soglia, incrementiamo i colpi
            if current_dist <= target_distance_cm:
                hits += 1
            else:
                # Se leggiamo un valore "lontano", resettiamo il contatore
                # Questo evita stop causati da singoli glitch del sensore
                hits = 0

            # Mantenimento traiettoria con giroscopio
            error = self.normalize_angle(self.get_yaw() - ref_angle)
            correction = round(self.kp_gyro * error)
            motor_pair.move(self.PAIR, -correction, velocity=velocity)

            await runloop.sleep_ms(20)

        motor_pair.stop(self.PAIR, stop=motor.SMART_BRAKE)
        print("Ostacolo confermato. Stop.")

#riassunto funzioni
# resetta angolo giroscopioasync def reset_gyro(self):
# va avanti robot.drive_straight_safe( distanza in cm, velocità)
# Gira il robot robot.turn(self, angolo, velocità, tipo di rotazione ) – i tipi di rotazione sono pivot e spin
# ruota motore non connesso a ruote accessory_move_degrees(self, angolo in gradi, velocità):



# --- PROGRAMMA PRINCIPALE ---

VEL = 720
BLACK = 0

async def main():
#Scrivete qui il vostro codice. Ogni volta che chiamate una funzione mettete await prima
    robot = RobotController(port.A, port.C, accessory_port=port.B)
    await robot.reset_gyro()
    await robot.accessory_move_degrees(27, velocity=100)
    await robot.drive_straight_safe(distance_cm=35.7, velocity=500)
    # task lancio
    await robot.accessory_move_degrees(90, velocity=2000)
    await robot.accessory_move_degrees(-80, velocity=700)
    await robot.accessory_move_degrees(80, velocity=2000)
    await robot.accessory_move_degrees(-80, velocity=900)
    await robot.accessory_move_degrees(80, velocity=2000)
    await robot.accessory_move_degrees(-80, velocity=90)
    await robot.accessory_move_degrees(80, velocity=2000)
    await robot.accessory_move_degrees(-45, velocity=900)
    await robot.turn(-20)
    await robot.drive_straight_safe(distance_cm=25, velocity=400)
    await robot.turn(50)
    await robot.drive_straight_safe(distance_cm=2, velocity=400)
    # task massi
    await robot.accessory_move_degrees(27, velocity=600)
    await robot.turn(-55)
    # task equilibrio
    await robot.accessory_move_degrees(35, velocity=600)
    await robot.drive_straight_safe(2.5, velocity= 400)
    await robot.turn(-70)
    # task buttare giu
    await robot.drive_straight_safe(-2, velocity= 400)
    await robot.turn(10)
    await robot.accessory_move_degrees(-100, velocity=600)
    await robot.drive_straight_safe(30, velocity=400)
    await robot.turn(-80)
    await robot.drive_straight_safe(-3, velocity=400)
    await robot.accessory_move_degrees(80, velocity=600)
    await robot.accessory_move_degrees(-80, velocity=600)
    #task balena
    await robot.drive_straight_safe(-1, velocity= 400)
    await robot.turn(40)
    await robot.drive_straight_safe(32, velocity= 400)
    await robot.turn(-14)
    await robot.accessory_move_degrees(80, velocity=2500)
    await robot.drive_straight_safe(3, velocity= 400)
    await robot.accessory_move_degrees(-160, velocity=2500)
    await robot.turn(60)
    await robot.drive_straight_safe(26, velocity=400)
    # task carrello
    await robot.accessory_move_degrees(-10, velocity=100)
    runloop.sleep_ms(200)
    #task buzzico
    await robot.turn(-30)
    await robot.drive_straight_safe(40, velocity=400)
    await robot.turn(-5)
    await robot.drive_straight_safe(3, velocity=400)
    await robot.accessory_move_degrees(-35,velocity=300)
    await robot.turn(90)
    await robot.turn(-120)
    await robot.drive_straight_safe(50, velocity=300)

    

    #await robot.accessory_move_degrees(-25, velocity=700)
    #await robot.accessory_move_degrees(40, velocity=100)
    
    #await robot.accessory_move_degrees(-100, velocity=1000)
'''
    await robot.accessory_move_degrees(-105, velocity=700)
    await robot.drive_straight_safe( distance_cm=30, velocity=400)
    await robot.turn(25)
    await robot.drive_straight_safe( distance_cm=-70, velocity=2000)
    '''
    

    
runloop.run(main()) #fine main

# Esempio: Avanzamento controllato fino a ostacolo
    #await robot.drive_until_distance(target_distance_cm=15, velocity=VEL)
    #await robot.turn(90)
    #await robot.drive_straight_safe(15, VEL)
    #await robot.accessory_move_degrees(-45, 400)
    #await robot.drive_until_color(VEL, BLACK)
    #await robot.line_follow_distance(distance_cm=100, velocity=200)

    # Esegue un quadrato
    #for _ in range(4):
    #    await robot.drive_straight_safe(30, velocity=VEL)    # Avanti 30cm
    #    await robot.turn(90)                                    # Gira 90 gradi
    # Deallocazione risorse hardware
   #motor_pair.unpair(motor_pair.PAIR_1)
