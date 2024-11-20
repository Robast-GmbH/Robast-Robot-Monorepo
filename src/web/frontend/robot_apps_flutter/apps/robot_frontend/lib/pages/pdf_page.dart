import 'package:flutter/material.dart';
import 'package:pdfrx/pdfrx.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/titled_view.dart';

class PdfPage extends StatelessWidget {
  const PdfPage({required this.path, super.key});

  final String path;

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      ignoreEmergencyStop: true,
      child: TitledView(
        title: path.split('/').last,
        showBackButton: true,
        child: PdfViewer.file(
          path,
          params: const PdfViewerParams(backgroundColor: RobotColors.secondaryBackground),
        ),
      ),
    );
  }
}
