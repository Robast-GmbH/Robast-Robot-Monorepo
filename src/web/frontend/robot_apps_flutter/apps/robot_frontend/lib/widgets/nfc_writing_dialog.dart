import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

class NFCWritingDialog extends StatefulWidget {
  const NFCWritingDialog({required this.nfcData, super.key});

  final String nfcData;

  @override
  State<NFCWritingDialog> createState() => _NFCWritingDialogState();
}

class _NFCWritingDialogState extends State<NFCWritingDialog> {
  bool writeSuccess = false;
  bool writeInProgress = true;

  Future<void> writeNFC() async {
    writeInProgress = true;
    setState(() {});

    final isSuccess = await Provider.of<UserProvider>(context, listen: false).writeNFC(
      robotName: 'rb_theron',
      nfcData: widget.nfcData,
    );
    if (isSuccess) {
      writeSuccess = true;
      writeInProgress = false;
    } else {
      writeSuccess = false;
      writeInProgress = false;
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
    if (writeInProgress) {
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
    } else if (writeSuccess) {
      return const Text('NFC Tag erfolgreich beschrieben');
    } else {
      return const Text('Fehler beim Beschreiben des NFC Tags');
    }
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('NFC Tag schreiben'),
      content: buildContent(),
      actions: <Widget>[
        if (writeSuccess)
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
            },
            child: const Text('OK'),
          )
        else if (!writeInProgress && !writeSuccess)
          TextButton(
            onPressed: writeNFC,
            child: const Text('Erneut versuchen'),
          ),
        if (!writeSuccess)
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
