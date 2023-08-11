# web_interface_flutter

A new Flutter project.

## Getting Started

This project is a starting point for a Flutter application.

A few resources to get you started if this is your first Flutter project:

- [Lab: Write your first Flutter app](https://docs.flutter.dev/get-started/codelab)
- [Cookbook: Useful Flutter samples](https://docs.flutter.dev/cookbook)

For help getting started with Flutter development, view the
[online documentation](https://docs.flutter.dev/), which offers tutorials,
samples, guidance on mobile development, and a full API reference.


# Anleitung zum Wechseln der Map

Schritt 1 
Im /assets Ordner RL_Tiplu_6.png mit neuer Map ersetzen, bei Namensänderung des Map-Assets nicht vergessen den Namen in der pubspec.yaml Datei anzupassen.

Schritt 2
Alle hardgecodeten Felder im robo_map.dart File bezüglich der Map ändern.
1. Offset mapOrigin = const Offset(x,y); aktualisieren, dafür neuen Ursprung mit der Resolution verrechnen
2. Falls nötig final double resolution = 0.05; anpassen
3. width und height auf Größe des neuen Map-Assets setzen

# Kompilieren für Linux

1. Kommando flutter build linux --release ausführen
2. Unter /build/linux/x64/release befindet sich nun die gebaute Anwendung in Form des Bundle Ordners
3. Falls die Anwendung nicht startet chmod +x und starten über Terminal via ./web_interface_flutter versuchen

# Als Webapp hosten

1. Kommando flutter run -d web-server --web-port 8080 --web-hostname 0.0.0.0 --release
2. Aufrufen via Host-IP:port
