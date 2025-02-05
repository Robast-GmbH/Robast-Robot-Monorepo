import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';

class ClockView extends StatefulWidget {
  const ClockView({
    super.key,
    this.fontSize = 32,
  });

  final double fontSize;

  @override
  State<ClockView> createState() => _ClockViewState();
}

class _ClockViewState extends State<ClockView> {
  late Stream<DateTime> dateTimeStream;

  @override
  void initState() {
    super.initState();
    dateTimeStream = Stream.periodic(const Duration(seconds: 1), (count) {
      return DateTime.now();
    }).asBroadcastStream();
  }

  @override
  Widget build(BuildContext context) {
    return StreamBuilder<DateTime>(
      stream: dateTimeStream,
      builder: (context, stream) {
        final date = stream.hasData ? stream.data! : DateTime.now();
        return Text(
          "${date.hour}:${date.minute < 10 ? "0${date.minute}" : date.minute}",
          textAlign: TextAlign.center,
          style: TextStyle(
            color: RobotColors.primaryText,
            fontSize: widget.fontSize,
          ),
        );
      },
    );
  }
}
