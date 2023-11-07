import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/widgets/manual_move_map.dart';
import 'package:web_interface_flutter/widgets/page_frame.dart';

class ManualMovePage extends StatefulWidget {
  const ManualMovePage({super.key});

  @override
  State<ManualMovePage> createState() => _ManualMovePageState();
}

class _ManualMovePageState extends State<ManualMovePage> {
  @override
  Widget build(BuildContext context) {
    return PageFrame(
      title: "Robotersteuerung",
      color: AppColors.turquoise,
      child: Container(
          margin: Constants.mediumPadding,
          decoration: BoxDecoration(
            color: AppColors.lightGrey,
            borderRadius: BorderRadius.circular(16),
          ),
          child: const ManualMoveMap()),
    );
  }
}
