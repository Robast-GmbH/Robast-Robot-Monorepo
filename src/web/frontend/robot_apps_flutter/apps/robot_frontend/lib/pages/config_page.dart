import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';

import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/home_page.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ConfigPage extends StatefulWidget {
  const ConfigPage({
    super.key,
    this.autoClose = false,
  });

  final bool autoClose;

  @override
  State<ConfigPage> createState() => _ConfigPageState();
}

class _ConfigPageState extends State<ConfigPage> {
  final _formKey = GlobalKey<FormState>();
  String robotBackendAddress = '';
  String middlewareAddress = '';
  late Future<SharedPreferences> getSharedPreferencesFuture;
  bool isFinished = false;

  Future<void> finishConfiguration({
    required String robotBackendAddress,
    required String middlewareAddress,
  }) async {
    Provider.of<RobotProvider>(context, listen: false).initRobotAPI(prefix: robotBackendAddress);
    MiddlewareApiUtilities().setPrefix(prefix: middlewareAddress);
    unawaited(Provider.of<MapProvider>(context, listen: false).fetchBuildingMap());
    Provider.of<ModuleProvider>(context, listen: false).stopSubmodulesUpdateTimer();
    Navigator.popUntil(context, ModalRoute.withName('/root'));
    await Navigator.push(
      context,
      MaterialPageRoute<HomePage>(builder: (context) => const HomePage()),
    );
  }

  @override
  void initState() {
    super.initState();
    getSharedPreferencesFuture = SharedPreferences.getInstance();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: BackgroundView(
        child: Row(
          children: [
            const Expanded(child: SizedBox()),
            Expanded(
              child: FutureBuilder<SharedPreferences>(
                future: getSharedPreferencesFuture,
                builder: (context, snapshot) {
                  if (snapshot.connectionState == ConnectionState.done) {
                    if (snapshot.data!.getString('robotBackendAddress') != null && snapshot.data!.getString('middlewareAddress') != null && widget.autoClose) {
                      WidgetsBinding.instance.addPostFrameCallback((_) async {
                        if (!isFinished) {
                          isFinished = true;
                          await finishConfiguration(
                            robotBackendAddress: snapshot.data!.getString('robotBackendAddress') ?? '',
                            middlewareAddress: snapshot.data!.getString('middlewareAddress') ?? '',
                          );
                        }
                      });
                      return const SizedBox();
                    }
                    return Form(
                      key: _formKey,
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: <Widget>[
                          Padding(
                            padding: const EdgeInsets.all(8),
                            child: TextFormField(
                              initialValue: snapshot.data!.getString('robotBackendAddress') ?? '',
                              decoration: const InputDecoration(
                                labelText: 'Enter the address of the robot backend',
                                hintText: 'http://ip:port',
                              ),
                              validator: (value) {
                                final ipPattern = RegExp(r'^http://\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}:\d{1,5}$');
                                if (!ipPattern.hasMatch(value!)) {
                                  return 'Please enter a valid URL in the format http://ip:port';
                                }
                                return null;
                              },
                              onSaved: (value) {
                                robotBackendAddress = value!;
                              },
                            ),
                          ),
                          Padding(
                            padding: const EdgeInsets.all(8),
                            child: TextFormField(
                              initialValue: snapshot.data!.getString('middlewareAddress') ?? '',
                              decoration: const InputDecoration(
                                labelText: 'Enter the address of the middleware',
                                hintText: 'http://ip:port',
                              ),
                              validator: (value) {
                                final ipPattern = RegExp(r'^http://\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}:\d{1,5}$');
                                if (!ipPattern.hasMatch(value!)) {
                                  return 'Please enter a valid URL in the format http://ip:port';
                                }
                                return null;
                              },
                              onSaved: (value) {
                                middlewareAddress = value!;
                              },
                            ),
                          ),
                          Padding(
                            padding: const EdgeInsets.all(8),
                            child: ElevatedButton(
                              onPressed: () async {
                                if (_formKey.currentState!.validate()) {
                                  _formKey.currentState!.save();
                                  await snapshot.data!.setString('robotBackendAddress', robotBackendAddress);
                                  await snapshot.data!.setString('middlewareAddress', middlewareAddress);
                                }
                                await finishConfiguration(
                                  robotBackendAddress: robotBackendAddress,
                                  middlewareAddress: middlewareAddress,
                                );
                              },
                              child: const Padding(
                                padding: EdgeInsets.symmetric(vertical: 8, horizontal: 32),
                                child: Text('Submit'),
                              ),
                            ),
                          ),
                        ],
                      ),
                    );
                  } else {
                    return const Center(child: CircularProgressIndicator());
                  }
                },
              ),
            ),
            const Expanded(child: SizedBox()),
          ],
        ),
      ),
    );
  }
}
