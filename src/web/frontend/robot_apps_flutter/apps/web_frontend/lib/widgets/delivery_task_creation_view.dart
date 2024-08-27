import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/widgets/custom_dropdown_view.dart';

class DeliveryTaskCreationView extends StatefulWidget {
  const DeliveryTaskCreationView({
    required this.controller,
    super.key,
  });

  final TaskCreationController controller;

  @override
  State<DeliveryTaskCreationView> createState() => _DeliveryTaskCreationViewState();
}

class _DeliveryTaskCreationViewState extends State<DeliveryTaskCreationView> {
  @override
  Widget build(BuildContext context) {
    final rmfProvider = Provider.of<RMFProvider>(context, listen: false);
    final robotProvider = Provider.of<FleetProvider>(context, listen: false);
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: ListView(
        children: [
          CustomDropdownView(
            value: widget.controller.pickupPlaceID,
            items: rmfProvider.getPickupLocations(),
            hint: 'Start auswählen',
            onChanged: (value) {
              setState(() {
                widget.controller.pickupPlaceID = value;
              });
            },
          ),
          CustomDropdownView(
            value: widget.controller.dropoffPlaceID,
            items: rmfProvider.getDropoffLocations(),
            hint: 'Ziel auswählen',
            onChanged: (value) {
              setState(() {
                widget.controller.dropoffPlaceID = value;
              });
            },
          ),
          CustomDropdownView(
            value: widget.controller.submoduleID,
            items: robotProvider.getIDsOfModules(robotName: 'rb_theron'),
            hint: 'Schublade auswählen',
            onChanged: (value) {
              setState(() {
                widget.controller.submoduleID = value;
              });
            },
          ),
        ],
      ),
    );
  }
}
