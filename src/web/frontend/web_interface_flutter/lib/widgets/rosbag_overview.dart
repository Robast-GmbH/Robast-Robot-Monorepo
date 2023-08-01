import 'package:flutter/material.dart';

class RosbagOverview extends StatefulWidget {
  const RosbagOverview({super.key});

  @override
  State<RosbagOverview> createState() => _RosbagOverviewState();
}

class _RosbagOverviewState extends State<RosbagOverview> {
  bool isRecording = false;
  @override
  Widget build(BuildContext context) {
    return Center(
      child: IconButton(
        icon: Icon(isRecording ? Icons.stop : Icons.play_arrow),
        iconSize: 64,
        onPressed: () {
          isRecording = !isRecording;
          setState(() {});
        },
      ),
    );
  }
}
