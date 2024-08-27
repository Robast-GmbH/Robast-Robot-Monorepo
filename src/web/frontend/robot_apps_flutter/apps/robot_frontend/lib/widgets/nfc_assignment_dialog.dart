import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

enum NFCAssignmentStatus {
  inProgress,
  success,
  error,
}

class NFCAssignmentDialog extends StatefulWidget {
  const NFCAssignmentDialog({required this.userID, super.key});

  final String userID;

  @override
  State<NFCAssignmentDialog> createState() => _NFCAssignmentDialogState();
}

class _NFCAssignmentDialogState extends State<NFCAssignmentDialog> {
  NFCAssignmentStatus status = NFCAssignmentStatus.inProgress;

  Future<void> readAndAssignNFC() async {
    status = NFCAssignmentStatus.inProgress;
    setState(() {});

    final isSuccess = await Provider.of<UserProvider>(context, listen: false).readAndAssignUserNFC(
      robotName: 'rb_theron',
      userID: widget.userID,
    );
    if (isSuccess) {
      status = NFCAssignmentStatus.success;
    } else {
      status = NFCAssignmentStatus.error;
    }
    if (mounted) {
      setState(() {});
    }
  }

  @override
  void initState() {
    super.initState();
    readAndAssignNFC();
  }

  Widget buildContent() {
    switch (status) {
      case NFCAssignmentStatus.inProgress:
        return const Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              'Bitte den NFC-Tag an das Gerät halten',
              style: TextStyle(color: RobotColors.secondaryText),
            ),
            SizedBox(
              height: 16,
            ),
            Center(child: CircularProgressIndicator()),
          ],
        );
      case NFCAssignmentStatus.success:
        return const Text(
          'NFC Tag erfolgreich zugewiesen',
          style: TextStyle(color: RobotColors.secondaryText),
        );
      case NFCAssignmentStatus.error:
        return const Text(
          'Fehler beim Zuweisen des NFC Tags',
          style: TextStyle(color: RobotColors.secondaryText),
        );
    }
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text(
        'NFC Tag zuweisen',
        style: TextStyle(color: RobotColors.secondaryText),
      ),
      content: buildContent(),
      actions: <Widget>[
        if (status == NFCAssignmentStatus.error)
          TextButton(
            onPressed: readAndAssignNFC,
            child: const Text(
              'Erneut versuchen',
              style: TextStyle(color: RobotColors.secondaryText),
            ),
          ),
        if (status == NFCAssignmentStatus.success)
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
            },
            child: const Text(
              'OK',
              style: TextStyle(color: RobotColors.secondaryText),
            ),
          )
        else
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
            },
            child: const Text(
              'Abbrechen',
              style: TextStyle(color: RobotColors.secondaryText),
            ),
          ),
      ],
    );
  }
}
