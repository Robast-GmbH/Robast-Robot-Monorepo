import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/controller/location_selection_controller.dart';
import 'package:web_frontend/models/provider/map_provider.dart';
import 'package:web_frontend/widgets/custom_dropdown_button.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

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
    final controller = widget.controller;
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              widget.label,
              textAlign: TextAlign.end,
              style: const TextStyle(fontSize: 24, color: WebColors.secondaryText),
            ),
            const SizedBox(
              width: 16,
            ),
            CustomDropdownButton(
              hint: 'Station auswählen',
              value: controller.station,
              onChanged: (value) {
                if (controller.station == value) return;
                setState(() {
                  controller.setStation(value ?? '');
                });
                widget.onChanged?.call();
              },
              items: Provider.of<MapProvider>(context).roomsByStations.keys.toList(),
            ),
            const SizedBox(
              width: 16,
            ),
            CustomDropdownButton(
              value: controller.room,
              hint: 'Raum auswählen',
              disabledHint: 'Bitte wählen Sie zuerst eine Station aus',
              onChanged: (value) {
                setState(() => controller.setRoom(value ?? ''));
                widget.onChanged?.call();
              },
              items: controller.station?.isEmpty ?? true ? [] : Provider.of<MapProvider>(context).roomsByStations[controller.station]!.toList(),
            ),
          ],
        ),
      ),
    );
  }
}
