import 'package:flutter/material.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class DrawerTile extends StatelessWidget {
  const DrawerTile({
    super.key,
    required this.robotName,
    required this.moduleID,
    required this.drawerID,
  });
  final String robotName;
  final int moduleID;
  final int drawerID;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Container(
        padding: const EdgeInsets.all(8),
        decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(16),
            boxShadow: const [
              BoxShadow(
                color: Colors.grey, // Shadow color
                offset: Offset(0, 2), // Changes position of shadow
                blurRadius: 2, // Changes size of shadow
                spreadRadius: 1, // Expands the shadow
              ),
            ],
            color: Colors.white),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: Text("Modul Nr. $moduleID"),
            ),
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  ElevatedButton(
                      onPressed: () async {
                        await APIService.openDrawer(robotName, moduleID, drawerID);
                      },
                      child: const Text("Öffnen")),
                  const SizedBox(
                    width: 8,
                  ),
                  ElevatedButton(
                    onPressed: () async {
                      await APIService.closeDrawer(robotName, moduleID, drawerID);
                    },
                    child: const Text("Schließen"),
                  ),
                ],
              ),
            )
          ],
        ),
      ),
    );
  }
}
