import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';
import 'package:robot_frontend/widgets/buttons/custom_dropdown_button.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class LocationSelector extends StatefulWidget {
  const LocationSelector({
    required this.controller,
    required this.label,
    this.onChanged,
    super.key,
  });

  final LocationSelectionController controller;
  final String label;
  final void Function()? onChanged;

  @override
  State<LocationSelector> createState() => _LocationSelectorState();
}

class _LocationSelectorState extends State<LocationSelector> {
  @override
  Widget build(BuildContext context) {
    final mapProvider = Provider.of<MapProvider>(context, listen: false);
    final controller = widget.controller;
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            Text(
              widget.label,
              textAlign: TextAlign.end,
              style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: CustomDropdownButton(
                  hint: 'Station auswählen',
                  value: controller.station,
                  onChanged: (value) {
                    if (controller.station == value) return;
                    setState(() {
                      controller.setStation(value ?? '');
                    });
                    widget.onChanged?.call();
                  },
                  items: mapProvider.roomsByStations.keys.toList()),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: CustomDropdownButton(
                value: controller.room,
                hint: 'Raum auswählen',
                onChanged: (value) {
                  setState(() => controller.setRoom(value ?? ''));
                  widget.onChanged?.call();
                },
                items: controller.station?.isEmpty ?? true ? [] : mapProvider.roomsByStations[controller.station]?.toList() ?? [],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
