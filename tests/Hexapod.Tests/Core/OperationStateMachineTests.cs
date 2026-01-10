using FluentAssertions;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Services;
using Hexapod.Core.StateMachine;
using Microsoft.Extensions.Logging.Abstractions;
using Xunit;

namespace Hexapod.Tests.Core;

public class OperationStateMachineTests
{
    private readonly OperationStateMachine _stateMachine;
    private readonly TestEventBus _eventBus;

    public OperationStateMachineTests()
    {
        _eventBus = new TestEventBus();
        _stateMachine = new OperationStateMachine(_eventBus, NullLogger<OperationStateMachine>.Instance);
    }

    [Fact]
    public void InitialMode_ShouldBeInitializing()
    {
        _stateMachine.CurrentMode.Should().Be(OperationMode.Initializing);
    }

    [Fact]
    public async Task TransitionTo_ValidTransition_ShouldSucceed()
    {
        // Initializing -> SemiAutonomous is valid
        var result = await _stateMachine.TransitionToAsync(OperationMode.SemiAutonomous, "Test transition");
        
        result.Should().BeTrue();
        _stateMachine.CurrentMode.Should().Be(OperationMode.SemiAutonomous);
    }

    [Fact]
    public async Task TransitionTo_InvalidTransition_ShouldFail()
    {
        // Initializing -> Autonomous is invalid (must go through SemiAutonomous first)
        var result = await _stateMachine.TransitionToAsync(OperationMode.Autonomous, "Test transition");
        
        result.Should().BeFalse();
        _stateMachine.CurrentMode.Should().Be(OperationMode.Initializing);
    }

    [Fact]
    public async Task ForceEmergencyStop_ShouldTransitionToEmergencyStop()
    {
        await _stateMachine.TransitionToAsync(OperationMode.SemiAutonomous, "Setup");
        
        await _stateMachine.ForceEmergencyStopAsync("Test emergency");
        
        _stateMachine.CurrentMode.Should().Be(OperationMode.EmergencyStop);
    }

    [Fact]
    public async Task ForceSafeMode_ShouldTransitionToSafeMode()
    {
        await _stateMachine.TransitionToAsync(OperationMode.SemiAutonomous, "Setup");
        
        await _stateMachine.ForceSafeModeAsync("Low battery");
        
        _stateMachine.CurrentMode.Should().Be(OperationMode.SafeMode);
    }

    [Theory]
    [InlineData(OperationMode.SemiAutonomous, OperationMode.Autonomous, true)]
    [InlineData(OperationMode.SemiAutonomous, OperationMode.RemoteControl, true)]
    [InlineData(OperationMode.Autonomous, OperationMode.SemiAutonomous, true)]
    [InlineData(OperationMode.RemoteControl, OperationMode.Autonomous, true)]
    [InlineData(OperationMode.SafeMode, OperationMode.Autonomous, false)]
    public async Task ValidTransitions_ShouldBeAllowed(
        OperationMode from, 
        OperationMode to, 
        bool expectedResult)
    {
        // Setup: Get to the 'from' state
        await _stateMachine.TransitionToAsync(OperationMode.SemiAutonomous, "Setup");
        if (from != OperationMode.SemiAutonomous)
        {
            await _stateMachine.TransitionToAsync(from, "Setup to from state");
        }

        var result = await _stateMachine.TransitionToAsync(to, "Test transition");
        
        result.Should().Be(expectedResult);
    }

    [Fact]
    public async Task CanTransitionTo_ShouldReturnCorrectValue()
    {
        await _stateMachine.TransitionToAsync(OperationMode.SemiAutonomous, "Setup");
        
        _stateMachine.CanTransitionTo(OperationMode.Autonomous).Should().BeTrue();
        _stateMachine.CanTransitionTo(OperationMode.Initializing).Should().BeFalse();
    }

    private class TestEventBus : IEventBus
    {
        public void Publish<TEvent>(TEvent @event) where TEvent : HexapodEvent { }
        public IObservable<TEvent> Subscribe<TEvent>() where TEvent : HexapodEvent 
            => System.Reactive.Linq.Observable.Empty<TEvent>();
        public IObservable<HexapodEvent> SubscribeAll()
            => System.Reactive.Linq.Observable.Empty<HexapodEvent>();
    }
}
