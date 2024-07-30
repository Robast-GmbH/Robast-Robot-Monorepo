import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

enum NFCWritingStatus {
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
  NFCWritingStatus status = NFCWritingStatus.inProgress;

  Future<void> writeNFC() async {
    status = NFCWritingStatus.inProgress;
    setState(() {});

    final isSuccess = await Provider.of<UserProvider>(context, listen: false).createAndWriteUserNFC(
      robotName: 'rb_theron',
      userID: widget.userID,
    );
    if (isSuccess) {
      status = NFCWritingStatus.success;
    } else {
      status = NFCWritingStatus.error;
    }
    if (mounted) {
      setState(() {});
    }
  }

  @override
  void initState() {
    super.initState();
    writeNFC();
  }

  Widget buildContent() {
    switch (status) {
      case NFCWritingStatus.inProgress:
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
      case NFCWritingStatus.success:
        return const Text('NFC Tag erfolgreich beschrieben');
      case NFCWritingStatus.error:
        return const Text('Fehler beim Beschreiben des NFC Tags');
    }
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('NFC Tag schreiben'),
      content: buildContent(),
      actions: <Widget>[
        if (status == NFCWritingStatus.error)
          TextButton(
            onPressed: writeNFC,
            child: const Text('Erneut versuchen'),
          ),
        if (status == NFCWritingStatus.success)
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
