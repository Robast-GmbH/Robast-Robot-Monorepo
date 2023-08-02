import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:url_launcher/url_launcher_string.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/pages/admin_page.dart';
import 'package:web_interface_flutter/pages/drawer_selection_page.dart';
import 'package:web_interface_flutter/pages/manual_control_page.dart';
import 'package:web_interface_flutter/widgets/robo_map.dart';

import 'login_page.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> {
  bool isManualControlOpened = false;
  final controller = MapController();
  double currentMapScale = 1.0;
  late Future<void> loadData;

  Offset scalePoint(Offset point, double scale) {
    return Offset(point.dx * scale, point.dy * scale);
  }

  @override
  void initState() {
    loadData = Provider.of<RobotProvider>(context, listen: false).updateProviderData();
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Karte"),
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
      ),
      endDrawer: Drawer(
        child: ListView(
          padding: EdgeInsets.zero,
          children: <Widget>[
            DrawerHeader(
                decoration: BoxDecoration(
                  color: Theme.of(context).colorScheme.inversePrimary,
                ),
                child: Image.asset("assets/Logo.png")),
            ListTile(
              leading: const Icon(Icons.admin_panel_settings),
              title: const Text('Admin'),
              onTap: () {
                Navigator.push(context, MaterialPageRoute(builder: (context) => const AdminPage()));
              },
            ),
            ListTile(
              leading: const Icon(Icons.settings),
              title: const Text('Manuelle Kontrolle'),
              onTap: () {
                Navigator.push(context, MaterialPageRoute(builder: (context) => const ManualControlPage()));
              },
            ),
            ListTile(
              leading: const Icon(Icons.text_snippet),
              title: const Text('Impressum'),
              onTap: () {
                launchUrlString("https://robast.de/impressum/");
              },
            ),
            ListTile(
              leading: const Icon(Icons.logout),
              title: const Text('Logout'),
              onTap: () {
                Navigator.pushReplacement(context, MaterialPageRoute(builder: (context) => const LoginPage()));
              },
            ),
          ],
        ),
      ),
      body: RoboMap(
        controller: controller,
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          if (controller.position != null) {
            Navigator.push(
              context,
              MaterialPageRoute(
                builder: (context) => TaskCreationPage(
                  targetPosition: scalePoint(controller.position!, currentMapScale),
                ),
              ),
            );
          } else {
            showDialog(
                context: context,
                builder: (context) => AlertDialog(
                      title: const Text("Keine Position ausgewählt!"),
                      content: const Text("Bitte Map anklicken, um eine Position auszuwählen."),
                      actions: [
                        TextButton(
                          child: const Text('Okay'),
                          onPressed: () {
                            Navigator.of(context).pop();
                          },
                        ),
                      ],
                    ));
          }
        },
        child: const Icon(Icons.add_task_rounded),
      ),
    );
  }
}
