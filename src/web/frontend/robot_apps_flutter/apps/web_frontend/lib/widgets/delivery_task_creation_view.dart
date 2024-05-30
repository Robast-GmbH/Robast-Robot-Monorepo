import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';
import 'package:web_frontend/models/provider/rmf_provider.dart';
import 'package:web_frontend/models/provider/robot_provider.dart';

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
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: ListView(
        children: [
          _buildDropdownButton(
            value: widget.controller.pickupNode,
            items: rmfProvider.getPickupLocations(),
            hint: 'Start auswählen',
            onChanged: (value) {
              setState(() {
                widget.controller.pickupNode = value;
              });
            },
          ),
          _buildDropdownButton(
            value: widget.controller.dropoffNode,
            items: rmfProvider.getDropoffLocations(),
            hint: 'Ziel auswählen',
            onChanged: (value) {
              setState(() {
                widget.controller.dropoffNode = value;
              });
            },
          ),
          _buildDropdownButton(
            value: widget.controller.drawerID,
            items: robotProvider.getIDsOfModules(robotName: 'rb_theron'),
            hint: 'Schublade auswählen',
            onChanged: (value) {
              setState(() {
                widget.controller.drawerID = value;
              });
            },
          ),
        ],
      ),
    );
  }

  Padding _buildDropdownButton({
    required String? value,
    required List<String> items,
    required String hint,
    required void Function(String?) onChanged,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: DropdownButton<String>(
        isExpanded: true,
        value: value,
        items: items.map((String value) {
          return DropdownMenuItem<String>(
            value: value,
            child: Text(value),
          );
        }).toList(),
        hint: Text(hint),
        onChanged: onChanged,
      ),
    );
  }
}
