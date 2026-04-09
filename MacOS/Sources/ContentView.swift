import SwiftUI

struct ContentView: View {
    @State private var state = AppState()
    private let timer = Timer.publish(every: 1.0 / 60.0, on: .main, in: .common).autoconnect()

    var body: some View {
        HSplitView {
            ControlsSidebar(state: state)
                .frame(minWidth: 250, idealWidth: 280, maxWidth: 350)

            MazeCanvas(state: state)
        }
        .onReceive(timer) { _ in
            state.tick()
        }
        .onAppear {
            state.setupKeyboard()
        }
        .onDisappear {
            state.teardownKeyboard()
        }
    }
}
