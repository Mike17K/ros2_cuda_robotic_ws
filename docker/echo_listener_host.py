import roslibpy
import time

# Σύνδεση στον Docker Server μέσω Tailscale IP
client = roslibpy.Ros(host="100.105.198.26", port=9090)


def receive_msg(message):
    print(f"[PC Λήψη] Μήνυμα από Docker: {message['data']}")


# Ορισμός του Topic που θέλουμε να ακούσουμε
listener = roslibpy.Topic(client, "/chatter", "std_msgs/msg/String")

print("Προσπάθεια σύνδεσης με τον ROS 2 Docker Server...")
client.run()

if client.is_connected:
    print("Συνδέθηκε επιτυχώς! Ακούω για μηνύματα...")
    listener.subscribe(receive_msg)
else:
    print("Αποτυχία σύνδεσης. Ελέγξτε αν τρέχει το Docker και το Tailscale.")

try:
    while client.is_connected:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nΤερματισμός...")
    client.terminate()
