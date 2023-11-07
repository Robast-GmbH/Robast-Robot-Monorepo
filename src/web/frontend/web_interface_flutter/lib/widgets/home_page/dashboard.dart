import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:intl/date_symbol_data_local.dart';
import 'package:intl/intl.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/constants/gaps.dart';
import 'package:web_interface_flutter/widgets/battery_level_indicator.dart';

import '../../constants/names.dart';

class Dashboard extends StatefulWidget {
  const Dashboard({super.key});

  @override
  State<Dashboard> createState() => _DashboardState();
}

class _DashboardState extends State<Dashboard> {
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
    return Column(
      children: [
        Container(
          padding: Constants.smallPadding + const EdgeInsets.only(top: 8),
          margin: Constants.smallPadding,
          decoration: BoxDecoration(borderRadius: BorderRadius.circular(32), color: AppColors.darkBlue),
          child: Column(
            children: [
              Column(
                children: [
                  if (!kIsWeb) ...[
                    Row(
                      children: [
                        Gaps.mediumHorizontal,
                        Expanded(
                          flex: 2,
                          child: buildDatetimeInfo(),
                        ),
                        Expanded(
                          child: buildStatusIndicatorBar(),
                        ),
                      ],
                    ),
                    Gaps.mediumVertical,
                  ],
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      const Text("Hey, ich bin", style: TextStyle(color: Colors.white, fontSize: 48, fontWeight: FontWeight.w300)),
                      Text(" ${names[Random().nextInt(names.length)]}",
                          style: const TextStyle(color: Colors.white, fontSize: 48, fontWeight: !kIsWeb ? FontWeight.w500 : FontWeight.w600)),
                    ],
                  ),
                  Gaps.tinyVertical,
                  const Text(
                    "Ein Roboter von Robast Robotic Assistant GmbH",
                    style: TextStyle(color: AppColors.white, fontSize: 12, fontWeight: FontWeight.w400),
                  ),
                  Gaps.smallVertical,
                ],
              ),
              Row(
                children: [
                  if (!kIsWeb) ...[
                    buildInfoBox(
                      title: "Error Log:",
                      content: '',
                      borderRadius: BorderRadius.circular(16),
                    ),
                  ],
                  buildInfoBox(
                    title: "Aktueller Auftrag:",
                    content: 'Standby',
                    borderRadius: BorderRadius.circular(16),
                    flex: 4,
                  ),
                  if (!kIsWeb) ...[
                    buildInfoBox(
                      title: "Nächster Auftrag:",
                      content: '',
                      borderRadius: BorderRadius.circular(16),
                    ),
                  ]
                ],
              )
            ],
          ),
        ),
      ],
    );
  }

  Row buildStatusIndicatorBar() {
    return Row(
      children: [
        const Expanded(child: SizedBox()),
        Container(
          padding: const EdgeInsets.symmetric(vertical: 2, horizontal: 4),
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(4),
            color: AppColors.darkBlueAccent,
          ),
          child: const BatteryLevelIndicator(),
        ),
        Gaps.mediumHorizontal,
      ],
    );
  }

  FutureBuilder<void> buildDatetimeInfo() {
    return FutureBuilder<void>(
      future: initializeDateFormatting("de_DE"),
      builder: (context, snapshot) {
        if (snapshot.connectionState == ConnectionState.done) {
          return StreamBuilder<DateTime>(
            stream: dateTimeStream,
            builder: (context, stream) {
              final date = stream.hasData ? stream.data! : DateTime.now();
              return Row(
                children: [
                  Expanded(
                      child: Text(
                    DateFormat('d. MMMM y').format(date),
                    style: const TextStyle(color: AppColors.white, fontWeight: FontWeight.w300),
                  )),
                  Expanded(
                    child: Text(
                      "${date.hour}:${date.minute < 10 ? "0${date.minute}" : date.minute}",
                      textAlign: TextAlign.center,
                      style: const TextStyle(color: AppColors.white, fontWeight: FontWeight.w300),
                    ),
                  ),
                ],
              );
            },
          );
        }
        return const SizedBox();
      },
    );
  }

  Widget buildInfoBox({
    required String title,
    required String content,
    int flex = 1,
    required BorderRadius borderRadius,
  }) {
    return Expanded(
      flex: flex,
      child: Container(
        height: 128,
        margin: Constants.smallPadding,
        padding: Constants.smallPadding,
        decoration: BoxDecoration(
          borderRadius: borderRadius,
          color: AppColors.darkBlueAccent,
        ),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Text(
              title,
              style: const TextStyle(
                color: AppColors.white,
                fontSize: 20,
              ),
            ),
            Gaps.smallVertical,
            Text(
              content,
              style: const TextStyle(
                color: AppColors.white,
                fontSize: 20,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
