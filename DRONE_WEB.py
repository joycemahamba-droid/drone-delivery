from dronekit import connect, VehicleMode, LocationGlobalRelative
from geopy.geocoders import Nominatim
import time
from math import radians, sin, cos, sqrt, atan2
import sys
import threading

from flask import Flask, request, jsonify
from flask_cors import CORS

# =========================
# ETATS PARTAGES
# =========================
etat_global = {
    "annuler_web": False
}
etat_drone = {
    "status": "en_attente"
}

missions = []  # liste des commandes reçues
NOMBRE_COMMANDES_MAX = 3

# =========================
# FONCTIONS UTILITAIRES
# =========================

def get_distance_metres(loc1, loc2):
    """Calcule la distance entre deux points GPS"""
    lat1, lon1, lat2, lon2 = map(radians, [loc1.lat, loc1.lon, loc2.lat, loc2.lon])
    dlon, dlat = lon2 - lon1, lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return 6371000 * c

def goto_position_target_global_relative(lat, lon, alt):
    """Envoie le drone à une position GPS relative"""
    vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))

def changer_altitude(vehicle, delta):
    """Modifie l'altitude cible et commande le drone"""
    global altitude_cible, dest_lat, dest_lon
    altitude_cible = max(1, altitude_cible + delta)
    print(f"\nNouvelle altitude cible → {altitude_cible:.1f} m")
    if 'dest_lat' in globals() and 'dest_lon' in globals():
        vehicle.simple_goto(LocationGlobalRelative(dest_lat, dest_lon, altitude_cible))

def verification_de_la_batterie():
    """Simule la vérification de la batterie"""
    if battery <= 20:
        print(f"\nBatterie faible ({battery}%). Retour à la base recommandé.")
        return True
    return False

def gestion_clavier(vehicle, etat):
    """Thread pour lire les commandes clavier"""
    print("""
Commandes clavier :
  u  -> URGENCE
  y  -> Annuler livraison
  +  -> Monter altitude
  -  -> Descendre altitude
  q  -> Quitter contrôle altitude
""")
    while not etat["stop"]:
        try:
            cmd = input().strip().lower()
            if cmd == "u":
                etat["urgence"] = True
                return
            elif cmd == "y":
                etat["annuler"] = True
                return
            elif cmd == "+":
                changer_altitude(vehicle, 1)
            elif cmd == "-":
                changer_altitude(vehicle, -1)
            elif cmd == "q":
                print("Fin du contrôle altitude")
        except EOFError:
            pass

# =========================
# SERVEUR FLASK
# =========================

app = Flask(__name__)
CORS(app)

@app.route("/commande", methods=["POST"])
def commande():
    data = request.get_json()
    action = data.get("action")

    if action == "continuer":
        etat_drone["status"] = "livraison continue"
        return jsonify({"status": "ok", "message": "On continue la livraison"}), 200

    if action == "annuler":
        etat_global["annuler_web"] = True
        etat_drone["status"] = "commande annulée (web)"
        return jsonify({"status": "ok", "message": "Commande annulée, on passe à l'adresse suivante"}), 200

    return jsonify({"status": "error", "message": "action inconnue"}), 400

@app.route("/info", methods=["GET"])
def info():
    if "dest_lat" not in globals() or "dest_lon" not in globals():
        return jsonify({
            "ok": False,
            "message": "Pas de destination en cours",
            "status": etat_drone["status"]
        }), 200

    current = vehicle.location.global_relative_frame
    dest = LocationGlobalRelative(dest_lat, dest_lon, altitude_cible)

    dist_m = get_distance_metres(current, dest)
    vitesse_ms = vehicle.airspeed if vehicle.airspeed else 1
    temps_s = dist_m / max(vitesse_ms, 0.1)

    return jsonify({
        "ok": True,
        "distance_km": round(dist_m / 1000, 2),
        "temps_min": round(temps_s / 60),
        "adresse": str(address),
        "status": etat_drone["status"]
    }), 200

@app.route("/demande_livraison", methods=["POST"])
def demande_livraison():
    global missions

    if len(missions) >= NOMBRE_COMMANDES_MAX:
        return jsonify({
            "status": "refuse",
            "message": f"Nombre maximum de commandes atteint ({NOMBRE_COMMANDES_MAX})"
        }), 200

    data = request.get_json()
    adresse_user = data.get("adresse")
    article = data.get("article")

    if not adresse_user:
        return jsonify({"status": "error", "message": "Adresse manquante"}), 400

    if 'geolocator' not in globals():
        return jsonify({"status": "error", "message": "Serveur non prêt (geolocator)"}), 500

    location = geolocator.geocode(adresse_user)
    if not location:
        return jsonify({"status": "refuse", "message": "Adresse introuvable"}), 200

    dest_lat_tmp, dest_lon_tmp = location.latitude, location.longitude

    # Règle : distance max 15 km depuis la base
    dist_base = get_distance_metres(
        home,
        LocationGlobalRelative(dest_lat_tmp, dest_lon_tmp, altitude_cible)
    )
    if dist_base > 15000:
        return jsonify({
            "status": "refuse",
            "message": "Adresse trop éloignée, choisissez un point de référence plus proche"
        }), 200

    # Ajouter la mission à la liste
    missions.append({
        "adresse": adresse_user,
        "lat": dest_lat_tmp,
        "lon": dest_lon_tmp,
        "article": article
    })

    etat_drone["status"] = f"{len(missions)}/{NOMBRE_COMMANDES_MAX} commandes reçues"
    print(f"Commande {len(missions)}/{NOMBRE_COMMANDES_MAX} : {adresse_user} ({article})")

    return jsonify({
        "status": "accepte",
        "message": f"Commande {len(missions)}/{NOMBRE_COMMANDES_MAX} acceptée",
        "adresse": adresse_user,
        "total": len(missions)
    }), 200

def lancer_serveur():
    app.run(host="0.0.0.0", port=5000, threaded=True)

# =========================
# CONNEXION AU DRONE
# =========================

print("Connexion au drone...")
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
print("Drone connecté!")

print("Lancement du serveur web Flask sur http://localhost:5000 ...")
serveur_thread = threading.Thread(target=lancer_serveur, daemon=True)
serveur_thread.start()

print("Recherche d'un signal GPS valide...")
while vehicle.gps_0.fix_type < 2:
    time.sleep(1)
print("GPS valide détecté!")

home = vehicle.location.global_frame
vehicle.home_location = home
battery = 100

# =========================
# CONFIGURATION DU VOL
# =========================

while True:
    try:
        altitude = float(input("Hauteur de vol souhaitée (m): "))
        speed = float(input("Vitesse de vol souhaitée (m/s): "))
        if altitude > 0 and speed > 0:
            break
    except ValueError:
        print("Entrée invalide.")

altitude_cible = altitude

vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode.name != "GUIDED":
    time.sleep(0.5)
vehicle.armed = True
while not vehicle.armed:
    time.sleep(0.5)

print(f"Décollage à {altitude}m...")
vehicle.airspeed = speed
vehicle.simple_takeoff(altitude)
while vehicle.location.global_relative_frame.alt < altitude * 0.95:
    time.sleep(1)
print("Altitude atteinte!\n")

# =========================
# ATTENTE DE 3 COMMANDES
# =========================

geolocator = Nominatim(user_agent="drone_locator", timeout=5)

print(f"En attente de {NOMBRE_COMMANDES_MAX} commandes des utilisateurs...")
while len(missions) < NOMBRE_COMMANDES_MAX:
    time.sleep(1)

print(f"\n{NOMBRE_COMMANDES_MAX} commandes reçues. Démarrage de la tournée...\n")

# =========================
# VISITE DES DESTINATIONS
# =========================

for mission in missions:
    address = mission["adresse"]
    dest_lat = mission["lat"]
    dest_lon = mission["lon"]
    article = mission["article"]

    print(f"\nProchaine livraison : {address} (Article: {article})")
    etat_drone["status"] = f"vers {address}"
    print(f"En route vers {address}...")
    goto_position_target_global_relative(dest_lat, dest_lon, altitude_cible)

    etat = {"urgence": False, "annuler": False, "stop": False}
    clavier_thread = threading.Thread(target=gestion_clavier, args=(vehicle, etat), daemon=True)
    clavier_thread.start()

    mission_stop = False
    cause_stop = None
    commande_annuler = False
    dernier_affi = 0

    while vehicle.mode.name == "GUIDED":
        if etat["urgence"]:
            print("\nUrgence opérateur !")
            vehicle.mode = VehicleMode("LOITER")
            mission_stop = True
            cause_stop = "urgence"
            etat_drone["status"] = "urgence opérateur"
            break

        if etat["annuler"]:
            print("\nLivraison annulée (clavier) ! Passage à l'adresse suivante.")
            commande_annuler = True
            etat_drone["status"] = f"commande vers {address} annulée (clavier)"
            break

        if etat_global["annuler_web"]:
            print("\nLivraison annulée (web) ! Passage à l'adresse suivante.")
            commande_annuler = True
            etat_global["annuler_web"] = False
            etat_drone["status"] = f"commande vers {address} annulée (web)"
            break

        if verification_de_la_batterie():
            print("Mission interrompue à cause de la batterie")
            mission_stop = True
            cause_stop = "batterie faible"
            etat_drone["status"] = "batterie faible"
            break

        dist = get_distance_metres(
            vehicle.location.global_relative_frame,
            LocationGlobalRelative(dest_lat, dest_lon, altitude_cible)
        )
        if time.time() - dernier_affi >= 2:
            print(f"\rDistance restante de {address}: {dist:.2f} m", end="")
            sys.stdout.flush()
            dernier_affi = time.time()

        if dist < 1:
            print(f"\n{address} atteinte.")
            etat_drone["status"] = f"livraison {address} atteinte"
            break

        time.sleep(1)

    if mission_stop and cause_stop == "urgence":
        print("Fin de la mission, passage en mode manuel...")
        vehicle.close()
        sys.exit(0)
    elif mission_stop and cause_stop == "batterie faible":
        print("Retour automatique à la base...")
        break
    if commande_annuler:
        continue

    print(f"Atterrissage pour la livraison à {address}...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(2)
    print("Livraison simulée...")
    time.sleep(5)

    # Redécollage pour la prochaine
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.5)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    print(f"Redécollage à {altitude_cible} m...")
    vehicle.simple_takeoff(altitude_cible)
    while vehicle.location.global_relative_frame.alt < altitude_cible * 0.95:
        time.sleep(1)
    print("Altitude atteinte!\n")

# =========================
# RETOUR A LA BASE
# =========================

print("\nRetour à la base (RTL)...")
vehicle.mode = VehicleMode("RTL")
etat_drone["status"] = "retour à la base"

while vehicle.armed:
    dist_home = get_distance_metres(vehicle.location.global_frame, home)
    alt = vehicle.location.global_relative_frame.alt
    print(f"\rDistance à la base: {dist_home:.1f} m | Altitude actuelle: {alt:.2f} m", end="")
    sys.stdout.flush()
    if dist_home < 1 and alt < 0.1:
        break
    time.sleep(1)

print("\nAtterrissage terminé.")
vehicle.close()
etat_drone["status"] = "au sol"
