import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

enum NFCAssignmentStatus {
  inProgress,
  success,
  error,
}

class NFCWritingDialog extends StatefulWidget {
  const NFCWritingDialog({required this.userID, super.key});

  final String userID;

  @override
  State<NFCWritingDialog> createState() => _NFCWritingDialogState();
}

class _NFCWritingDialogState extends State<NFCWritingDialog> {
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
            Text('Bitte den NFC-Tag an das Gerät halten'),
            SizedBox(
              height: 16,
            ),
            Center(child: CircularProgressIndicator()),
          ],
        );
      case NFCAssignmentStatus.success:
        return const Text('NFC Tag erfolgreich zugewiesen');
      case NFCAssignmentStatus.error:
        return const Text('Fehler beim Zuweisen des NFC Tags');
    }
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('NFC Tag zuweisen'),
      content: buildContent(),
      actions: <Widget>[
        if (status == NFCAssignmentStatus.error)
          TextButton(
            onPressed: readAndAssignNFC,
            child: const Text('Erneut versuchen'),
          ),
        if (status == NFCAssignmentStatus.success)
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
            },
            child: const Text('OK'),
          )
        else
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
            },
            child: const Text('Abbrechen'),
          ),
      ],
    );
  }
}
