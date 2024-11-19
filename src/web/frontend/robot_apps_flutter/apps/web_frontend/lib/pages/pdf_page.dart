import 'package:flutter/material.dart';
import 'package:pdfrx/pdfrx.dart';

class PdfPage extends StatelessWidget {
  const PdfPage({required this.path, super.key});

  final String path;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(
          path.split('/').last,
        ),
      ),
      body: PdfViewer.uri(Uri.parse(path)),
    );
  }
}
