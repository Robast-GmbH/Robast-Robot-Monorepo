import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class NextCleaningView extends StatelessWidget {
  const NextCleaningView({required this.robotName, super.key});
  final String robotName;
  @override
  Widget build(BuildContext context) {
    final hygieneApi = MiddlewareApiUtilities().hygiene;
    return FutureBuilder(
      future: hygieneApi.getNextCleaning(robotName: robotName),
      builder: (context, snapshot) {
        if (snapshot.connectionState == ConnectionState.waiting) {
          return const Text('Lädt...');
        }
        if (snapshot.hasError || snapshot.data == null) {
          return const Text('Fehler beim Laden der Reinigungsinformation');
        }
        return Text('Nächste erforderliche Reinigung: ${DateFormat('dd.MM.yyyy HH:mm').format(snapshot.data!)} Uhr');
      },
    );
  }
}
