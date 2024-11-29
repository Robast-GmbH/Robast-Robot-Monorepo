import 'package:flutter/material.dart';
import 'package:pdfrx/pdfrx.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class PdfPage extends StatelessWidget {
  const PdfPage({
    required this.path,
    this.inactivityTimerEnabled = true,
    super.key,
  });

  final String path;
  final bool inactivityTimerEnabled;

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      inactivityTimerEnabled: inactivityTimerEnabled,
      title: path.split('/').last.split('.').first,
      child: PdfViewer.file(
        path,
        params: const PdfViewerParams(backgroundColor: RobotColors.secondaryBackground),
      ),
    );
  }
}
