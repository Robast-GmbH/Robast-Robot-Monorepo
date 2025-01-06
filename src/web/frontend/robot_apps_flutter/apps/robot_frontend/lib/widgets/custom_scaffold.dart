import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/clock_view.dart';
import 'package:robot_frontend/widgets/emergency_stop_view.dart';
import 'package:robot_frontend/widgets/status_indicator_view.dart';
import 'package:robot_frontend/widgets/titled_view.dart';
import 'package:virtual_keyboard_custom_layout/virtual_keyboard_custom_layout.dart';

class CustomScaffold extends StatelessWidget {
  const CustomScaffold({
    super.key,
    this.child = const SizedBox(),
    this.showBackButton = true,
    this.inactivityTimerEnabled = true,
    this.onBackButtonPressed,
    this.title = '',
    this.ignoreMissingEmergencyStopData = false,
    this.collapsedTitle = false,
  });

  final Widget child;
  final bool showBackButton;
  final bool inactivityTimerEnabled;
  final VoidCallback? onBackButtonPressed;
  final String title;
  final bool ignoreMissingEmergencyStopData;
  final bool collapsedTitle;

  @override
  Widget build(BuildContext context) {
    print('rebuild CustomScaffold');
    return Scaffold(
      body: BackgroundView(
        inactivityTimerEnabled: inactivityTimerEnabled,
        child: Selector<RobotProvider, bool?>(
          selector: (context, robotProvider) => robotProvider.isEmergencyStopPressed,
          builder: (context, isEmergencyStopPressed, selectorChild) {
            if (isEmergencyStopPressed == null && !ignoreMissingEmergencyStopData) {
              return Column(
                children: [
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Expanded(
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.start,
                          children: [
                            Padding(
                              padding: const EdgeInsets.only(left: 16, top: 8),
                              child: IconButton(
                                padding: const EdgeInsets.all(0),
                                onPressed: () async {
                                  final robotProvider = Provider.of<RobotProvider>(context, listen: false);
                                  robotProvider.blockNavigation();
                                  await Navigator.push(context, MaterialPageRoute<ManualsPage>(builder: (context) => const ManualsPage()));
                                  robotProvider.unblockNavigation();
                                },
                                color: RobotColors.primaryIcon,
                                icon: const Icon(
                                  Icons.info_outline,
                                  size: 64,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                      const Padding(
                        padding: EdgeInsets.all(8),
                        child: ClockView(),
                      ),
                      const Expanded(
                        child: Padding(
                          padding: EdgeInsets.only(top: 8, right: 16),
                          child: StatusIndicatorView(
                            shouldBlockNavigation: true,
                          ),
                        ),
                      ),
                    ],
                  ),
                  const Expanded(
                    child: Center(
                        child: Padding(
                      padding: EdgeInsets.symmetric(horizontal: 256),
                      child: Text(
                        "Not-Aus Status konnte nicht abgerufen werden",
                        style: TextStyle(color: RobotColors.primaryText, fontSize: 72),
                        textAlign: TextAlign.center,
                      ),
                    )),
                  ),
                ],
              );
            }
            if (isEmergencyStopPressed ?? false) {
              Provider.of<KeyboardProvider>(context, listen: false).key = null;
              Navigator.of(context).popUntil((route) => route is PageRoute);
              return const EmergencyStopView();
            }
            return Stack(
              children: [
                GestureDetector(
                  onTap: () => Provider.of<KeyboardProvider>(context, listen: false).key = null,
                  child: TitledView(
                    title: title,
                    showBackButton: showBackButton,
                    onBackButtonPressed: onBackButtonPressed,
                    collapsedTitle: collapsedTitle,
                    child: child,
                  ),
                ),
                if (Provider.of<KeyboardProvider>(context).key != null)
                  Align(
                    alignment: Alignment.bottomCenter,
                    child: ColoredBox(
                      color: Colors.white,
                      child: VirtualKeyboard(
                        type: VirtualKeyboardType.Alphanumeric,
                        fontSize: 32,
                        height: 400,
                        defaultLayouts: const [VirtualKeyboardDefaultLayouts.English],
                        textController: TextEditingController(text: Provider.of<KeyboardProvider>(context, listen: false).text),
                        onKeyPress: (key) {
                          print('key:');
                          final provider = Provider.of<KeyboardProvider>(context, listen: false);
                          if (key.keyType == VirtualKeyboardKeyType.Action) {
                            switch (key.action) {
                              case VirtualKeyboardKeyAction.Backspace:
                                print('backspace and ${provider.text}');
                                if (provider.text!.isNotEmpty) {
                                  provider.text = provider.text!.substring(0, provider.text!.length - 1);
                                }
                                break;
                              case VirtualKeyboardKeyAction.Return:
                                provider.text = provider.text! + '\n';
                                break;
                              case VirtualKeyboardKeyAction.Space:
                                provider.text = provider.text! + ' ';
                                break;
                              case VirtualKeyboardKeyAction.Shift:
                                break;
                              default:
                                print('pressed unsupported action');
                                break;
                            }
                          } else if (key is VirtualKeyboardKey) {
                            print(key.text);
                            provider.text = provider.text! + key.text!;
                          }
                          provider.setTextState!();
                        },
                      ),
                    ),
                  ),
              ],
            );
          },
        ),
      ),
    );
  }
}
