import 'dart:convert';

import 'package:file_picker/file_picker.dart';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:http_parser/http_parser.dart' as parser;
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:web_frontend/pages/pdf_page.dart';

class ManualsManagementPage extends StatefulWidget {
  const ManualsManagementPage({super.key});

  @override
  State<ManualsManagementPage> createState() => _ManualsManagementPageState();
}

class _ManualsManagementPageState extends State<ManualsManagementPage> {
  final headers = {
    'accept': 'application/json',
    'Content-Type': 'application/json',
  };
  final manuals = <String>[];
  late Future<void> _loadFiles;

  String? _statusMessage;

  Future<void> pickAndUploadFile() async {
    try {
      // Step 1: Pick a file
      final result = await FilePicker.platform.pickFiles(
        type: FileType.custom,
        allowedExtensions: ['pdf'], // Only allow PDF files
      );

      if (result == null || result.files.isEmpty) {
        setState(() {
          _statusMessage = 'No file selected.';
        });
        return;
      }

      // Extract file data
      final file = result.files.first;
      final fileBytes = file.bytes;
      final fileName = file.name;

      if (fileBytes == null) {
        setState(() {
          _statusMessage = 'Failed to read the file.';
        });
        return;
      }

      final uri = Uri.parse('${MiddlewareApiUtilities().prefix}/manuals/');
      final request = http.MultipartRequest('POST', uri)
        ..files.add(
          http.MultipartFile.fromBytes(
            'file',
            fileBytes,
            filename: fileName,
            contentType: parser.MediaType('application', 'pdf'),
          ),
        );

      final response = await request.send();

      if (response.statusCode == 200 || response.statusCode == 201) {
        setState(() {
          _statusMessage = "File '$fileName' uploaded successfully.";
        });
      } else {
        setState(() {
          _statusMessage = 'Failed to upload. Server responded with status: ${response.statusCode}.';
        });
      }
    } catch (e) {
      setState(() {
        _statusMessage = 'An error occurred: $e';
      });
    }
  }

  Future<void> loadFiles() async {
    final manualsList = await MiddlewareApiUtilities().manuals.fetchManualsList();
    if (manualsList != null) {
      manuals
        ..clear()
        ..addAll(manualsList.map((e) => (e['name']) as String));
      setState(() {});
    }
  }

  @override
  void initState() {
    super.initState();
    _loadFiles = loadFiles();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Anleitungsmanagement'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () async {
              _loadFiles = loadFiles();
            },
          )
        ],
      ),
      floatingActionButton: FloatingActionButton(
          child: const Icon(Icons.add),
          onPressed: () async {
            await pickAndUploadFile();
            _loadFiles = loadFiles();
          }),
      body: FutureBuilder(
          future: _loadFiles,
          builder: (context, snapshot) {
            if (snapshot.connectionState != ConnectionState.done) {
              return const Center(child: CircularProgressIndicator());
            }
            return ListView(
              children: manuals
                  .map(
                    (manual) => ListTile(
                      title: Text(manual),
                      trailing: IconButton(
                        icon: const Icon(Icons.delete),
                        onPressed: () async {
                          await http.delete(
                            Uri.parse('${MiddlewareApiUtilities().prefix}/manuals/$manual'),
                            headers: headers,
                          );
                          _loadFiles = loadFiles();
                        },
                      ),
                      onTap: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute<PdfPage>(
                            builder: (context) => PdfPage(
                              path: '${MiddlewareApiUtilities().prefix}/manuals/$manual',
                            ),
                          ),
                        );
                      },
                    ),
                  )
                  .toList(),
            );
          }),
    );
  }
}
