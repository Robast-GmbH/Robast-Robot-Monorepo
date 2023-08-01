import 'package:flutter/material.dart';

class StatusOverview extends StatefulWidget {
  const StatusOverview({super.key});

  @override
  State<StatusOverview> createState() => _StatusOverviewState();
}

class _StatusOverviewState extends State<StatusOverview> {
  int percentage = 83;
  @override
  Widget build(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(32),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text("Batterielevel bei $percentage%"),
            const SizedBox(
              height: 8,
            ),
            LinearProgressIndicator(
              minHeight: 32,
              value: percentage / 100,
            )
          ],
        ),
      ),
    );
  }
}
