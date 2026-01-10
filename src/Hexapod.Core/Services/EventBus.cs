using System.Reactive.Linq;
using System.Reactive.Subjects;
using Hexapod.Core.Events;
using Microsoft.Extensions.Logging;

namespace Hexapod.Core.Services;

/// <summary>
/// Provides a reactive event bus for system-wide event distribution.
/// </summary>
public interface IEventBus
{
    /// <summary>
    /// Publishes an event to all subscribers.
    /// </summary>
    void Publish<TEvent>(TEvent @event) where TEvent : HexapodEvent;

    /// <summary>
    /// Subscribes to events of a specific type.
    /// </summary>
    IObservable<TEvent> Subscribe<TEvent>() where TEvent : HexapodEvent;

    /// <summary>
    /// Subscribes to all events.
    /// </summary>
    IObservable<HexapodEvent> SubscribeAll();
}

/// <summary>
/// Implementation of the event bus using System.Reactive.
/// </summary>
public sealed class EventBus : IEventBus, IDisposable
{
    private readonly Subject<HexapodEvent> _eventSubject = new();
    private readonly ILogger<EventBus> _logger;
    private bool _disposed;

    public EventBus(ILogger<EventBus> logger)
    {
        _logger = logger;
    }

    public void Publish<TEvent>(TEvent @event) where TEvent : HexapodEvent
    {
        if (_disposed)
        {
            throw new ObjectDisposedException(nameof(EventBus));
        }

        _logger.LogDebug("Publishing event {EventType}: {EventId}", 
            typeof(TEvent).Name, @event.EventId);
        
        _eventSubject.OnNext(@event);
    }

    public IObservable<TEvent> Subscribe<TEvent>() where TEvent : HexapodEvent
    {
        return System.Reactive.Linq.Observable.OfType<TEvent>(_eventSubject);
    }

    public IObservable<HexapodEvent> SubscribeAll()
    {
        return _eventSubject.AsObservable();
    }

    public void Dispose()
    {
        if (!_disposed)
        {
            _eventSubject.Dispose();
            _disposed = true;
        }
    }
}
