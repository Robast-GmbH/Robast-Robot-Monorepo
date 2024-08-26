import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';
import 'package:web_frontend/models/dropoff_place_submodule_assignment.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/widgets/custom_dropdown_view.dart';

class MultiDropOffTaskCreationView extends StatefulWidget {
  const MultiDropOffTaskCreationView({
    required this.controller,
    super.key,
  });

  final TaskCreationController controller;

  @override
  State<MultiDropOffTaskCreationView> createState() => _MultiDropOffTaskCreationViewState();
}

class _MultiDropOffTaskCreationViewState extends State<MultiDropOffTaskCreationView> {
  @override
  Widget build(BuildContext context) {
    final rmfProvider = Provider.of<RMFProvider>(context, listen: false);
    final robotProvider = Provider.of<FleetProvider>(context, listen: false);
    return ListView(
      children: List<Widget>.generate(widget.controller.dropoffPlaceSubmoduleAssignments.length, (index) {
            final assignment = widget.controller.dropoffPlaceSubmoduleAssignments[index];
            return Card(
              margin: const EdgeInsets.all(16),
              elevation: 5,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16),
                child: Column(
                  children: [
                    CustomDropdownView(
                      value: assignment.dropoffPlaceID,
                      items: rmfProvider.getDropoffLocations(),
                      hint: 'Ziel auswählen',
                      onChanged: (value) {
                        if (value != null) {
                          setState(() {
                            widget.controller.dropoffPlaceSubmoduleAssignments[index].dropoffPlaceID = value;
                          });
                        }
                      },
                    ),
                    CustomDropdownView(
                      value: assignment.submoduleID,
                      items: robotProvider.getIDsOfModules(robotName: 'rb_theron'),
                      hint: 'Schublade auswählen',
                      onChanged: (value) {
                        if (value != null) {
                          setState(() {
                            widget.controller.dropoffPlaceSubmoduleAssignments[index].submoduleID = value;
                          });
                        }
                      },
                    ),
                  ],
                ),
              ),
            );
          }).toList() +
          [
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 16),
              child: Center(
                child: Container(
                  decoration: const BoxDecoration(
                    color: Colors.blue,
                    shape: BoxShape.circle,
                  ),
                  child: IconButton(
                    icon: const Icon(Icons.add),
                    onPressed: () {
                      widget.controller.dropoffPlaceSubmoduleAssignments.add(DropoffPlaceSubmoduleAssignment());
                      setState(() {});
                    },
                  ),
                ),
              ),
            ),
          ],
    );
  }
}
