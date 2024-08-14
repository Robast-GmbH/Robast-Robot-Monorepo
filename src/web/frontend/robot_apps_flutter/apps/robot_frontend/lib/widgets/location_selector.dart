import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';

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
    return Card(
      color: Colors.white.withOpacity(0.4),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            Text(
              widget.label,
              textAlign: TextAlign.end,
              style: const TextStyle(
                fontSize: 24,
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: DropdownButton<String>(
                isExpanded: true,
                hint: const Text(
                  'Station auswählen',
                  style: TextStyle(fontSize: 24),
                ),
                value: controller.station,
                onChanged: (value) {
                  if (controller.station == value) return;
                  setState(() {
                    controller.setStation(value ?? '');
                  });
                  widget.onChanged?.call();
                },
                items: Provider.of<MapProvider>(context)
                    .roomsByStations
                    .keys
                    .map(
                      (availableStation) => DropdownMenuItem<String>(
                        value: availableStation,
                        alignment: Alignment.center,
                        child: Text(
                          availableStation,
                          style: const TextStyle(fontSize: 24),
                        ),
                      ),
                    )
                    .toList(),
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: DropdownButton<String>(
                isExpanded: true,
                value: controller.room,
                hint: const Text(
                  'Raum auswählen',
                  style: TextStyle(fontSize: 24),
                ),
                disabledHint: const Text(
                  '',
                  style: TextStyle(fontSize: 24),
                ),
                onChanged: (value) => setState(() => controller.setRoom(value ?? '')),
                items: controller.station?.isEmpty ?? true
                    ? []
                    : Provider.of<MapProvider>(context)
                        .roomsByStations[controller.station]!
                        .map(
                          (availableRoom) => DropdownMenuItem<String>(
                            value: availableRoom,
                            alignment: Alignment.center,
                            child: Text(
                              availableRoom,
                              style: const TextStyle(fontSize: 24),
                            ),
                          ),
                        )
                        .toList(),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
